# Multi-Robot Formation and Coordination

This challenge controls three simulated TurtleBot3 robots as one squad. Each
robot has independent sensors, localization, transforms, Nav2 servers, and
action endpoints:

[Watch the multi-agent challenge demonstration](https://drive.google.com/file/d/1YZhPHhTBXrxJ_z_175PIq7aj66sDhzLE/view?usp=sharing).

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

The demonstration uses the indoor `test_zone` model from the
[Gazebo models and worlds collection](https://github.com/leonhartyao/gazebo_models_worlds_collection),
pinned to revision `cce115b82691b7c529a02f47e5efa391145a4ca1`. Its mesh and
texture retain the upstream GPL-3.0 license. All three robots start together at
the window-side dock, facing into the house. Robot 1 is assigned to the large
main room, Robot 2 to the lower room, and Robot 3 to the upper room before they
return to the same
dock. The Nav2 map is generated from a 20 cm cross-section of the same
collision mesh at launch.

## Run

Start the three-robot stack through the shared challenge runner:

```bash
./scripts/run.sh --multi-agent start
```

Add `--gui` to open Gazebo. Preview the credential-free mission:

```bash
./scripts/run.sh --multi-agent preview --planner fake \
  "Form a wedge at the dock, inspect separate rooms, and regroup home."
```

Run it and approve the displayed plan:

```bash
./scripts/run.sh --multi-agent run --planner fake \
  "Form a wedge at the dock, inspect separate rooms, and regroup home."
```

Gemini can be selected after setting `GEMINI_API_KEY` in the ignored `.env`:

```bash
./scripts/run.sh --multi-agent run --planner gemini \
  "Form a wedge at the dock, inspect separate rooms, and regroup home."
```

`OMOKAI_DEMO_MAX_SPEED_MPS` sets the accepted simulation speed ceiling. The
default is `0.18`; a local simulation recording can raise it to `0.40` m/s.

Check the latest result or cancel an active mission from another terminal:

```bash
./scripts/run.sh --multi-agent status
./scripts/run.sh --multi-agent cancel
```

Stop the environment with:

```bash
./scripts/run.sh --multi-agent stop
```

The original `run_multi_robot.sh` wrapper remains supported.

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
- AMCL initializes each map pose; live `map -> robotN/base_link` transforms are
  combined on `/omokai_fleet/poses`. Missing pose state blocks dispatch, and
  measured separation below 0.30 m cancels the active squad. This is an
  emergency near-contact stop; ordinary spacing is managed by queued shared
  corridor reservations.
- A right-of-way arbiter sits after the three Nav2 collision monitors. It
  predicts close crossing paths, yields lower-priority velocity commands, and
  rotates priority every five seconds to prevent starvation.
- Shared-dock departure goals are queued in closest-first order. The first
  robot starts immediately; live map poses release the next queued robot after
  its predecessor has travelled 1.0 m from the dock. On return, arrivals remain
  serialized and the outer slots fill before the centre/apex slot so a parked
  robot cannot block another robot's approach corridor.
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
sections. Formation and regroup retain squad barriers. During split execution,
each robot owns an immutable work sequence and advances as soon as its required
reservation is free. A dock-departure reservation clears after 1.0 m of live
travel, allowing safe staggered motion instead of waiting for a complete room
visit. Other goal dependencies still require normal Nav2 completion.

The split phase sends Robot 1 to a holding point in the upper-right of the
large main room, clear of the enclosed-room return corridor. Robot 2 goes to
the lower enclosed room and Robot 3 to the upper enclosed room. They depart
from one window-side dock and regroup at the same three separated dock poses. Distance
ranking produces a deterministic assignment. Shared dock/corridor reservations
serialize convergence, while non-conflicting private route work proceeds
without a sibling barrier. A failure on a private reservation skips that
robot's remaining work and permits safe survivors to finish; a shared-corridor
failure, separation violation, or operator cancellation stops the fleet.
The demonstration uses a continuous leader-relative controller for Robot 2 and
Robot 3 while Robot 1 follows the Nav2 formation path. Local Nav2 controllers
avoid static obstacles, while the fleet layer owns squad separation and
fail-stop policy.
Nav2 behavior trees retain per-robot planning and recovery responsibility;
fleet right-of-way is centralized because a single-robot behavior tree cannot
negotiate ownership of a shared corridor.

Run all ROS-independent regression tests with:

```bash
./scripts/run_slam.sh check
```

If startup fails, inspect `docker compose logs robot`, stop, and retry.
