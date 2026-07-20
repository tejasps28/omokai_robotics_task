# Multi-Robot Formation and Coordination

This challenge controls three simulated TurtleBot3 robots as one squad. Each
robot has independent sensors, localization, transforms, Nav2 servers, and
action endpoints:

```text
validated squad intent
        |
formation geometry + route allocation
        |
synchronized fleet lifecycle
        |
 /robot1/navigate_to_pose
 /robot2/navigate_to_pose
 /robot3/navigate_to_pose
```

The language model selects only bounded squad policy: line or wedge formation,
the known inspection route, spacing, speed, split, and regroup. Coordinates,
robot ownership, synchronization, and motion remain deterministic.

## Run

Start the three-robot stack:

```bash
./scripts/run_multi_robot.sh start
```

Add `--gui` to open Gazebo. Preview the credential-free mission:

```bash
./scripts/run_multi_robot.sh preview --planner fake \
  "You three split the inspection route in a wedge and regroup home."
```

Run it and approve the displayed plan:

```bash
./scripts/run_multi_robot.sh run --planner fake \
  "You three split the inspection route in a wedge and regroup home."
```

Gemini can be selected after setting `GEMINI_API_KEY` in the ignored `.env`:

```bash
./scripts/run_multi_robot.sh run --planner gemini \
  "You three split the inspection route in a wedge and regroup home."
```

Check the latest result or cancel an active mission from another terminal:

```bash
./scripts/run_multi_robot.sh status
./scripts/run_multi_robot.sh cancel
```

Stop the environment with:

```bash
./scripts/run_multi_robot.sh stop
```

## Expected result

The successful demonstration passes these barriers:

```text
forming
formation_moving
executing_split
regrouping
succeeded
```

Every phase records one correlated outcome for robot1, robot2, and robot3.
Evidence is written under `runtime/artifacts/<mission-id>/` and ignored by
Git.

## Safety and failure handling

- Generated formation and regroup goals must maintain at least 0.50 m planned
  separation.
- Namespaced AMCL poses are monitored during motion; measured separation below
  0.38 m cancels the active squad.
- All three Nav2 action servers must be ready before any goal is dispatched.
- One robot failure or timeout cancels all unresolved goals.
- Operator cancellation waits for Nav2 confirmation from every active robot.
- Per-batch goal deadlines and a cumulative mission deadline are bounded.
- Hosted-provider failure sends no robot goal; the fake planner remains
  available offline.

## Architecture notes

All robots share the global `map` frame. Robot-local frames are unique:
`robotN/odom`, `robotN/base_footprint`, `robotN/base_link`, and prefixed sensor
frames. Topics and actions use `/robotN/...` namespaces.

Formation offsets are defined relative to a reference pose and rotated into
the map frame. Ordered route points are divided into balanced contiguous
sections. Synchronization barriers prevent a later phase from starting until
all three current results succeed.

The demonstration uses discrete synchronized formation waypoints, not a
continuous low-level formation controller. Local Nav2 controllers avoid static
obstacles, while the fleet layer owns squad separation and fail-stop policy.

Run all ROS-independent regression tests with:

```bash
./scripts/run_slam.sh check
```

If startup fails, inspect `docker compose logs robot`, stop, and retry.
