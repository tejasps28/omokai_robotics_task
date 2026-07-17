# Natural-Language Ground Robot

This project turns a natural-language instruction into a bounded robot mission
and executes it on a simulated TurtleBot3:

```text
prompt -> LLM proposal -> JSON validation -> deterministic executor -> Nav2 -> Gazebo
```

The language model proposes intent only. It cannot publish ROS messages or
choose arbitrary coordinates. A local validator accepts only known routes and
safe parameters, then a deterministic executor sends the corresponding poses
to Nav2.

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

Use `--planner openai` to select the OpenAI adapter instead of Gemini.

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
- `omokai_bringup`: Gazebo, AMCL, Nav2, and live mission runner.

## Documentation

- [Architecture](docs/architecture.md)
- [Mission format](docs/mission_format.md)
- [Sources and licenses](docs/sources.md)
- [Approach to the additional challenges](docs/future_work.md)

## License

Original code in this repository is Apache-2.0. Third-party components retain
their own licenses; see [sources](docs/sources.md).
