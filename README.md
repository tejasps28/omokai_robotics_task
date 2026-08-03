# Natural-Language Ground Robot and Autonomous SLAM

This project turns a natural-language instruction into a bounded robot mission
and executes it on a simulated TurtleBot3:

```text
prompt -> LLM proposal -> JSON validation -> deterministic executor -> Nav2 -> Gazebo
```

The language model proposes intent only. It cannot publish ROS messages or
choose arbitrary coordinates. A local validator accepts only known routes and
safe parameters, then a deterministic executor sends the corresponding poses
to Nav2.

The repository also includes the SLAM challenge: the same robot can build a
map from live LiDAR data, autonomously explore deterministic frontiers, save
the map, restart with AMCL localization, and navigate named locations.

The multi-robot challenge runs three namespaced TurtleBots with independent
Nav2 stacks. They form a line or wedge, split a trusted route, monitor
separation, and regroup.

The vision challenge adds local person detection, aligned RGB-D localization,
operator snapshots, and deterministic stand-off following.

## Requirements

- Linux with Docker Engine and Docker Compose v2
- approximately 15 GB of free disk space for the first build
- X11 if the Gazebo window is required
- a Gemini or OpenAI API key for the hosted-LLM demonstration

ROS and Gazebo do not need to be installed on the host.

## Quick start

Copy the environment template and add one API key:

```bash
cp .env.example .env
```

For Gemini:

```text
GEMINI_API_KEY=your-key
```

For OpenAI:

```text
OPENAI_API_KEY=your-key
```

The demonstration uses Gemini, but either provider can be selected.

Start Gazebo, localization, and Nav2:

```bash
./scripts/start.sh --gui
```

Run a mission:

```bash
./scripts/run_task1.sh \
  "Patrol the inspection loop once and return home."
```

The terminal displays the proposed JSON, validation result, and compiled route.
Approve it to move the robot. Stop the environment with:

```bash
./scripts/stop.sh
```

Headless mode uses the same stack:

```bash
./scripts/start.sh
```

For a credential-free demonstration, select the deterministic planner:

```bash
./scripts/run_task1.sh --planner fake \
  "Patrol the inspection loop twice and return home."
```

Use `--planner openai` or `--planner gemini` to select the corresponding
provider adapter.

## Example missions

```text
Patrol clockwise once, then patrol anticlockwise once, then return home.
Drive through the lanes once and return to start.
Sweep the full area once and return home.
```

The current catalog contains three routes:

- `inspection_loop`: a four-corner perimeter route;
- `aisle_sweep`: a serpentine pass through the two central aisles;
- `full_area_sweep`: horizontal passes, both central aisles, and outer
  corridors.

Missions may contain ordered forward/reverse or clockwise/counterclockwise
segments, with at most ten total traversals.

## What to expect

An accepted mission produces:

- the original prompt and provider-labelled JSON proposal;
- structural and semantic validation results;
- the exact ordered Nav2 goals;
- an append-only execution event stream;
- the final state and completed-goal count.

These files are written under `runtime/artifacts/<mission-id>/` and are ignored
by Git. Invalid or unsafe proposals are rejected before the executor starts.

## Repository layout

```text
docker/    container image and health check
scripts/   start, run, and stop commands
src/       ROS 2 packages and unit tests
docs/      architecture, mission format, sources, and future work
```

The source is split by responsibility:

- `omokai_interfaces`: immutable mission types and JSON Schema;
- `omokai_mission`: LLM adapters, validation, route catalog, and compiler;
- `omokai_executor`: deterministic state machine and Nav2 adapter;
- `omokai_pipeline`: application orchestration and operator CLI;
- `omokai_exploration`: deterministic frontier selection and exploration
  lifecycle;
- `omokai_fleet`: squad validation, formation geometry, route allocation,
  synchronized execution, and cancellation;
- `omokai_bringup`: Gazebo, SLAM Toolbox, AMCL, Nav2, and live runners.

## One runner for all three senior challenges

SLAM, multi-agent, and perception modes use the same Dockerfile, image,
Compose service, runtime directories, and stop command. Select the challenge
with one flag; internally the selected ROS launch file is the only simulation
entrypoint that changes:

```bash
./scripts/run.sh --slam start --gui
./scripts/run.sh --multi-agent start --gui
./scripts/run.sh --perception start --gui
```

Each flag also routes the challenge-specific operations:

```bash
./scripts/run.sh --slam explore --exploration-id slam-demo
./scripts/run.sh --multi-agent run --planner fake \
  "You three split the inspection route in a wedge and regroup home."
./scripts/run.sh --perception run \
  --mission-id moving-person-demo --coat-color white --yes
```

Only one challenge mode can be active at a time. The existing
`run_slam.sh`, `run_multi_robot.sh`, and `run_vision.sh` wrappers remain
available for compatibility.

## SLAM challenge

The complete headless workflow is:

```bash
./scripts/run_slam.sh start
./scripts/run_slam.sh explore --exploration-id slam-demo
./scripts/run_slam.sh save-map slam-demo
./scripts/run_slam.sh localize slam-demo
./scripts/run_slam.sh verify --mission-id saved-map-demo
```

Add `--gui` to the `start` or `localize` command to open Gazebo and RViz.
See the [SLAM demonstration guide](docs/slam.md) for status, cancellation,
expected output, architecture, and limitations.

## Multi-robot challenge

Start three robots headlessly and run the credential-free demonstration:

```bash
./scripts/run_multi_robot.sh start
./scripts/run_multi_robot.sh run --planner fake \
  "You three split the inspection route in a wedge and regroup home."
```

Use `--gui` with `start` for Gazebo, or `--planner gemini` for hosted
natural-language interpretation. See the
[multi-robot demonstration guide](docs/multi_robot.md).

## Vision/perception challenge

The default perception scene is a lightweight office layout adapted from
`office_small`, with a continuously walking white-coat actor. The actor begins
outside the parked robot's camera view; an approved mission first scans, then
detects and follows:

```bash
./scripts/run.sh --perception start --gui
./scripts/run.sh --perception run \
  --mission-id vision-demo --coat-color white --yes
```

Use `--stationary-actor` on `start` only for detector and identity regression
tests. See the [vision demonstration guide](docs/vision.md).

## Documentation

- [Architecture](docs/architecture.md)
- [Mission format](docs/mission_format.md)
- [SLAM and autonomous navigation](docs/slam.md)
- [Multi-robot formation and coordination](docs/multi_robot.md)
- [Vision target detection and following](docs/vision.md)
- [Sources and licenses](docs/sources.md)
- [Approach to the additional challenges](docs/future_work.md)

## License

Original code in this repository is Apache-2.0. Third-party components retain
their own licenses; see [sources](docs/sources.md).

## Video Submission of Task 1

[Task 1 demonstration video](https://drive.google.com/file/d/1Ue0LoBxiX2IKtk8JnhjX3pfpcOllZ7zF/view?usp=sharing)

The video demonstrates the complete Task 1 pipeline with TurtleBot3 in Gazebo.
At 02:30, the deterministic fake planner rejects a vaguely worded command;
after switching to Gemini, the instruction “go for an inspection around once
and return home” is interpreted and executed successfully.

- The planner produces a strict mission schema containing the route
  (`inspection_loop`, `aisle_sweep`, or `full_area_sweep`), ordered segments
  (`clockwise`/`counterclockwise` or `forward`/`reverse`), repetitions, speed,
  and return-home choice.
- Local validation accepts only safe schema-compliant missions, then the
  deterministic compiler converts the selected catalog route into ordered Nav2
  goals for execution.

## Video Submissions of the Senior Challenges

- [SLAM challenge demonstration video](https://drive.google.com/file/d/1fsejQN0ZJ2RPXQ1CLiTKOEt4595RYH2N/view?usp=drive_link)
- [Multi-agent challenge demonstration video](https://drive.google.com/file/d/1YZhPHhTBXrxJ_z_175PIq7aj66sDhzLE/view?usp=sharing)
- [Vision challenge demonstration video](https://drive.google.com/file/d/1KCIeKczialVr-4yUAyPIIrKJiAoVF8QM/view?usp=drive_link)

In the vision recording, the robot detects the person wearing a white coat and
approaches them. The Gazebo actor remains stationary because actor motion was
not functioning reliably during the recording.
