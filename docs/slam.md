# SLAM and Autonomous Navigation

This extension lets the simulated TurtleBot3 map an unknown arena, choose
frontier goals without manual waypoints, save the resulting occupancy grid,
and navigate named locations after restarting in localization mode.

```text
lidar + odometry + TF
        |
   SLAM Toolbox  --> live /map
        |
frontier selector --> Nav2 NavigateToPose
        |
    map saver --> map_server + AMCL --> named goals
```

The frontier selector is deterministic. It clusters known-free cells adjacent
to unknown space, removes small or unsafe clusters, scores information gain
against travel distance, and sends only a free map-frame pose to Nav2. Failed
regions are temporarily blacklisted. Goal timeout, mission timeout,
cancellation, failure count, and frontier exhaustion are explicit terminal
conditions.

## Run the demonstration

No API key is required for this challenge. Start online mapping headlessly:

```bash
./scripts/run_slam.sh start
```

Add `--gui` to open Gazebo and RViz. In a second terminal, start autonomous
exploration:

```bash
./scripts/run_slam.sh explore --exploration-id slam-demo
```

The robot continues until no eligible frontier remains or a configured safety
bound is reached. Inspect progress or request cancellation from another
terminal:

```bash
./scripts/run_slam.sh status slam-demo
./scripts/run_slam.sh cancel
```

Save the completed live map:

```bash
./scripts/run_slam.sh save-map slam-demo
```

The ignored `runtime/maps/` directory will contain `slam-demo.yaml` and its
PGM image. Restart with that map and verify two named goals:

```bash
./scripts/run_slam.sh localize slam-demo
./scripts/run_slam.sh verify --mission-id saved-map-demo
```

For a visible localization run, use
`./scripts/run_slam.sh localize slam-demo --gui`. Stop all containers with:

```bash
./scripts/run_slam.sh stop
```

## Expected results

A complete exploration ends with output similar to:

```text
state=completed, reason=no_frontiers, completed_goals=9, failed_goals=0
```

The exact frontier coordinates can vary slightly with simulation timing.
Completion, safety limits, and goal selection remain deterministic for the
same map and robot pose. Saved-map verification should end with:

```text
state=succeeded, completed_goals=2/2
```

Run the ROS-independent regression suite with:

```bash
./scripts/run_slam.sh check
```

Status, event, and result files are written below `runtime/artifacts/` and are
not committed.

## Packages and frames

- `omokai_exploration` owns occupancy-grid geometry, frontier selection, the
  exploration lifecycle, and its ROS coordinator.
- `omokai_bringup` composes Gazebo, SLAM Toolbox or map server/AMCL, Nav2, RViz,
  and saved-map verification.
- `map` is the global SLAM/localization frame, `odom` is locally continuous,
  and `base_footprint` is the robot frame used for navigation.
- SLAM mode produces `map -> odom`; saved-map mode uses AMCL to estimate it.

The language model is not part of exploration or motion control. It may
eventually select a validated high-level operation such as “map this area,”
but mapping goals and velocity control remain local and deterministic.

## Limitations and troubleshooting

- The demonstration targets a static indoor 2D LiDAR environment.
- Frontier exhaustion means no eligible reachable boundary remains; unknown
  space outside closed walls is intentionally left unknown.
- The two verification locations are defined for the supplied simulation
  arena and maps created from its default start pose.
- If startup does not reach healthy state, inspect
  `docker compose logs robot`, then run `./scripts/run_slam.sh stop` and retry.
- GUI mode requires working host X11 access; headless mode exercises the same
  mapping and navigation stack without the Gazebo or RViz clients.
