# Natural-Language Ground Robot and Vision Following

This project turns a natural-language instruction into a bounded robot mission
and executes it on a simulated TurtleBot3:

```text
prompt -> LLM proposal -> JSON validation -> deterministic executor -> Nav2 -> Gazebo
```

The language model proposes intent only. It cannot publish ROS messages or
choose arbitrary coordinates. A local validator accepts only known routes and
safe parameters, then a deterministic executor sends the corresponding poses
to Nav2.

The standalone vision challenge adds local person detection, aligned RGB-D
localization, operator snapshots, and deterministic stand-off following. The
operator can request a white, red, or unconstrained coat colour while rejected
people remain marked in red and cannot command motion.

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
- `omokai_perception`: YOLOX inference, coat selection, tracking, localization,
  and snapshots;
- `omokai_following`: mission coordination, active search, safety, and bounded
  visual following;
- `omokai_bringup`: Gazebo, the aligned RGB-D scene, actors, and visualization.

## Vision challenge

Start the moving white-coat actor and optional red distractor in Gazebo/RViz:

```bash
./scripts/run_vision.sh start --gui --red-actor
./scripts/run_vision.sh run \
  --mission-id vision-demo --target-class person \
  --coat-color white --standoff 1.2 --max-speed 0.38 --yes
```

Use `status`, `cancel`, and `stop` through the same script. See the
[vision guide](docs/vision.md) for expected behavior and limitations.

## Documentation

- [Architecture](docs/architecture.md)
- [Mission format](docs/mission_format.md)
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

## Video Submission of the Vision Challenge

Vision challenge video: `VIDEO_LINK_PENDING`

The recording will show bounded search from an initially parked robot,
white-versus-red target selection, the acquisition notification and snapshot,
aligned RGB-D localization, following, stand-off, and safe cancellation.
