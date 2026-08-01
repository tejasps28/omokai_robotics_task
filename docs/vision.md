# Vision Target Detection and Following

The perception mode runs an isolated TurtleBot3 demonstration in a minimal
office layout adapted from `office_small`, with one aligned RGB-D camera, a walking human actor, local person
detection, temporal target confirmation, metric depth localization, operator
snapshots, and a bounded deterministic follow controller.

The language model is not part of perception or motion control. Target
selection, image-to-range conversion, safety checks, and velocity commands are
local deterministic code.

## Start the environment

Build and start the normal moving-target GUI through the consolidated runner:

```bash
./scripts/run.sh --perception start --gui
```

The white-coat actor starts behind the parked robot and follows a closed path
through the office at 0.28 m/s by default. Its animation and motion parameters
are adapted from the pinned Black Coffee Robotics actor-plugin example. The
patrol waits for the ROS-to-Gazebo bridge subscriber before publishing its
first path, avoiding the startup race that could previously leave it in place.
Use the explicit stationary mode only for controlled detector tests:

```bash
./scripts/run.sh --perception start --gui --stationary-actor
```

Add the local red-coat distractor for attribute and identity testing:

```bash
./scripts/run.sh --perception start --gui --stationary-actor --red-actor
```

The red-coat actor is a packaged derivative of the pinned Apache-2.0 Black
Coffee Robotics actor asset, so `--red-actor` has no host-runtime asset
dependency. Its source and licence metadata are installed with the model. The
older `run_vision.sh` wrapper and `--moving-actor` flag remain supported.

Omit `--gui` for headless operation. Stop with:

```bash
./scripts/run.sh --perception stop
```

Use the consolidated task flag when starting this demo. Bare
`./scripts/start.sh --gui` intentionally starts the default/core navigation
environment and does not provide the vision mission services.

## Preview and run a mission

Preview validates and prints the exact bounded mission without moving:

```bash
./scripts/run.sh --perception preview \
  --target-class person \
  --coat-color white \
  --standoff 1.2 \
  --max-speed 0.38 \
  --mission-id vision-demo
```

Run the same mission and approve it interactively:

```bash
./scripts/run.sh --perception run \
  --target-class person \
  --coat-color white \
  --standoff 1.2 \
  --max-speed 0.38 \
  --mission-id vision-demo
```

Add `--yes` only for scripted acceptance. Inspect or cancel with:

```bash
./scripts/run.sh --perception status
./scripts/run.sh --perception cancel
```

## Observable interfaces

- `/robot1/camera/image`: aligned RGB source
- `/robot1/camera/depth_image`: metric depth source
- `/robot1/camera/camera_info`: camera intrinsics
- `/vision/detections`: all detected people
- `/vision/target_detection`: only the currently confirmed target
- `/vision/detections/image`: annotated operator image
- `/vision/selection_status`: attribute and confirmation state
- `/vision/follow_status`: follow state, reason, and latest observation
- `/vision/operator_event`: exactly-once acquisition snapshot event
- `/robot1/cmd_vel`: bounded velocity output owned by vision mode

Accepted targets are drawn in green. People that fail or do not conclusively
pass the requested attribute gate are drawn in red and cannot produce a target
observation.

## Safety behavior

A single frame cannot start motion. A person must pass the configured
attribute gate across multiple spatially consistent frames. The tracker keeps
an identity anchor and will not jump to a spatially unrelated person.

After operator approval, the parked robot performs a bounded in-place scan at
0.40 rad/s until the target is confirmed; it does not drive blindly through
the room. Forward and angular velocity are capped. The coordinator checks the protected
depth sector continuously while a mission is active, independently of whether
a target detection is currently available. It publishes an emergency zero
command when an obstacle is inside the safety distance. Forward motion also
stops at the configured stand-off, when observations become stale, on cancel,
on timeout, and on terminal failure. Initial target loss publishes a full zero
command; later bounded rotation may try to reacquire the same identity.
Prolonged loss fails and returns to zero velocity.

On the first confirmed sighting in an acquisition episode, original and
annotated PNG files plus correlated JSON metadata are stored under:

```text
runtime/artifacts/<mission-id>/vision/
```

Mission plan and result JSON files are stored beside that directory.

## Current limitations

- The detector currently exposes only the COCO `person` class.
- White, red, and unconstrained (`any`) coat gates are implemented with a
  lighting-tolerant HSV heuristic, not a learned re-identification model.
- The red-coat actor is a packaged test derivative, not an upstream production
  asset. Its UV mapping should be rechecked after asset or renderer changes.
- Gazebo actors are scripted visuals rather than dynamic collision bodies.
- The minimal office shell is adapted from the GPL-3.0 `office_small` world in the
  cited Gazebo models/worlds collection; its exact source and revision are
  recorded in [sources.md](sources.md). Furniture and decorative assets are
  omitted from the live scene to keep the perception corridor clear. The
  floor, four outer walls, and one short office partition use primitive
  collisions and explicit Harmonic-compatible SDF colors.
- The follow controller is intended for the dedicated vision world, where
  Nav2 is not simultaneously publishing velocity.
