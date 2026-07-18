"""Connect live SLAM maps and TF poses to autonomous frontier navigation."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic
from typing import Optional, Sequence

import rclpy
from nav_msgs.msg import OccupancyGrid as OccupancyGridMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_srvs.srv import Trigger

from .artifacts import ExplorationArtifactWriter
from .frontier import FrontierConfig
from .grid import measure_map_progress
from .ros_bridge import (
    ExplorationNav2Adapter,
    grid_from_message,
    robot_xy_from_transform,
)
from .session import (
    ExplorationConfig,
    ExplorationSession,
    ExplorationState,
)


def _default_exploration_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'slam-{timestamp}'


class ExplorationCoordinator(Node):
    """Serialize map, TF, Nav2, and timeout events through one session."""

    def __init__(
        self,
        *,
        exploration_id: str,
        config: ExplorationConfig,
        speed_mps: float,
        server_timeout_sec: float,
        artifact_root: Path,
    ) -> None:
        super().__init__(
            'omokai_exploration',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self._navigation = ExplorationNav2Adapter(
            self,
            speed_mps=speed_mps,
        )
        self._session = ExplorationSession(
            exploration_id,
            self._navigation,
            config=config,
        )
        self._artifacts = ExplorationArtifactWriter(
            artifact_root,
            exploration_id,
        )
        self._persisted_event_count = 0
        self._navigation.bind_outcomes(self._session)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGridMessage,
            '/map',
            self._on_map,
            map_qos,
        )
        self._cancel_service = self.create_service(
            Trigger,
            '~/cancel',
            self._on_cancel,
        )
        self._latest_map: Optional[OccupancyGridMessage] = None
        self._map_version = 0
        self._processed_map_version = 0
        self._started = False
        self._done = False
        self._exit_code = 1
        self._server_deadline = monotonic() + server_timeout_sec
        self._last_state = self._session.state
        self._timer = self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f'Exploration {exploration_id} waiting for Nav2 and a SLAM map'
        )
        self._flush_artifacts()

    @property
    def done(self) -> bool:
        return self._done

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def request_cancel(self) -> bool:
        return self._session.request_cancel()

    def _on_cancel(self, _request: Trigger.Request, response: Trigger.Response):
        accepted = self.request_cancel()
        response.success = accepted
        response.message = (
            'cancellation requested'
            if accepted
            else f'cannot cancel while state is {self._session.state.value}'
        )
        self._flush_artifacts()
        return response

    def _on_map(self, message: OccupancyGridMessage) -> None:
        self._latest_map = message
        self._map_version += 1

    def _tick(self) -> None:
        if self._done:
            return
        if not self._started:
            if not self._navigation.server_is_ready():
                if monotonic() >= self._server_deadline:
                    self.get_logger().error(
                        'Nav2 NavigateToPose server did not become ready'
                    )
                    self._done = True
                    self._flush_artifacts()
                return
            self._started = True
            self._session.start()
            self.get_logger().info('Nav2 is ready; exploration started')

        if (
            self._session.state is ExplorationState.WAITING_FOR_MAP
            and self._latest_map is not None
            and self._map_version > self._processed_map_version
        ):
            try:
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    Time(),
                )
                robot_x, robot_y = robot_xy_from_transform(transform)
                grid = grid_from_message(self._latest_map)
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f'Cannot evaluate the latest map yet: {error}',
                    throttle_duration_sec=2.0,
                )
            else:
                self._processed_map_version = self._map_version
                progress = measure_map_progress(grid)
                self.get_logger().info(
                    f'Map progress: known_area={progress.known_area_m2:.2f}m^2, '
                    f'known={progress.known_fraction:.1%}, '
                    f'unknown_cells={progress.unknown_cells}'
                )
                dispatched = self._session.observe_map(
                    grid,
                    robot_x,
                    robot_y,
                )
                if dispatched:
                    candidate = self._session.active_candidate
                    assert candidate is not None
                    self.get_logger().info(
                        f'Frontier goal dispatched: '
                        f'x={candidate.goal_x:.3f}, '
                        f'y={candidate.goal_y:.3f}, '
                        f'score={candidate.score:.3f}'
                    )

        if not self._session.state.terminal:
            self._session.tick()
        if self._session.state is not self._last_state:
            self.get_logger().info(
                f'Exploration state: {self._last_state.value} -> '
                f'{self._session.state.value}'
            )
            self._last_state = self._session.state
        self._flush_artifacts()
        if self._session.state.terminal:
            self._finalize()

    def _finalize(self) -> None:
        result = self._session.result
        assert result is not None
        self._exit_code = (
            0 if result.state is ExplorationState.COMPLETED else 1
        )
        self._done = True
        self._flush_artifacts()
        self._artifacts.write_result(result)
        self.get_logger().info(
            f'Exploration finished: state={result.state.value}, '
            f'reason={result.reason}, completed_goals={result.completed_goals}, '
            f'failed_goals={result.failed_goals}'
        )

    def _flush_artifacts(self) -> None:
        events = self._session.events
        if len(events) > self._persisted_event_count:
            self._artifacts.append_events(
                events[self._persisted_event_count :]
            )
            self._persisted_event_count = len(events)
        self._artifacts.write_status(self._session.snapshot)


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Explore the live SLAM map using deterministic frontiers.'
    )
    parser.add_argument('--exploration-id', default=_default_exploration_id())
    parser.add_argument(
        '--artifact-root',
        type=Path,
        default=Path('/data/artifacts'),
    )
    parser.add_argument('--speed-mps', type=float, default=0.15)
    parser.add_argument('--server-timeout-sec', type=float, default=30.0)
    parser.add_argument('--goal-timeout-sec', type=float, default=120.0)
    parser.add_argument('--mission-timeout-sec', type=float, default=900.0)
    parser.add_argument('--map-timeout-sec', type=float, default=30.0)
    parser.add_argument('--cancel-timeout-sec', type=float, default=10.0)
    parser.add_argument('--completion-confirmations', type=int, default=3)
    parser.add_argument('--max-failed-goals', type=int, default=8)
    parser.add_argument(
        '--max-goals',
        type=int,
        default=0,
        help='Stop successfully after this many goals; zero means unlimited.',
    )
    parser.add_argument('--min-cluster-size', type=int, default=5)
    parser.add_argument('--clearance-m', type=float, default=0.25)
    parser.add_argument('--blacklist-radius-m', type=float, default=0.5)
    parser.add_argument('--visited-radius-m', type=float, default=0.6)
    options = parser.parse_args(argv)
    if options.server_timeout_sec <= 0.0:
        parser.error('--server-timeout-sec must be greater than zero')
    if options.speed_mps <= 0.0:
        parser.error('--speed-mps must be greater than zero')
    if options.max_goals < 0:
        parser.error('--max-goals must not be negative')
    return options


def main(argv: Optional[Sequence[str]] = None) -> int:
    options = _parse_args(argv)
    try:
        config = ExplorationConfig(
            frontier=FrontierConfig(
                min_cluster_size=options.min_cluster_size,
                clearance_m=options.clearance_m,
                blacklist_radius_m=options.blacklist_radius_m,
                visited_radius_m=options.visited_radius_m,
            ),
            goal_timeout_sec=options.goal_timeout_sec,
            mission_timeout_sec=options.mission_timeout_sec,
            map_timeout_sec=options.map_timeout_sec,
            cancel_timeout_sec=options.cancel_timeout_sec,
            completion_confirmations=options.completion_confirmations,
            max_failed_goals=options.max_failed_goals,
            max_completed_goals=options.max_goals or None,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init()
    node = ExplorationCoordinator(
        exploration_id=options.exploration_id,
        config=config,
        speed_mps=options.speed_mps,
        server_timeout_sec=options.server_timeout_sec,
        artifact_root=options.artifact_root,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.get_logger().warning('Exploration interrupted by operator')
        node.request_cancel()
        deadline = monotonic() + config.cancel_timeout_sec
        while rclpy.ok() and not node.done and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
