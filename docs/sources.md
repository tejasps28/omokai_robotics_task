# Sources and Licenses

| Source | Tested version | License | Use |
|---|---|---|---|
| [ROS Docker image](https://hub.docker.com/_/ros) | `ros:jazzy-ros-base-noble`, pinned by digest in the Dockerfile | Package-specific; ROS core is primarily Apache-2.0 | Container base |
| [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) | `turtlebot3_navigation2` 2.3.6 | Apache-2.0 | Navigation parameters and map |
| [TurtleBot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) | 2.3.7 | Apache-2.0 | Robot models, spawn launch, and simulation launch reference |
| [Navigation2](https://github.com/ros-navigation/navigation2) | 1.3.12 | Apache-2.0 | Localization, planning, control, behavior trees, and navigation action |
| [ros_gz](https://github.com/gazebosim/ros_gz) | 1.0.22 | Apache-2.0 | ROS 2 and Gazebo integration |
| [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) | Harmonic packages supplied through ROS Jazzy | Apache-2.0 | Simulator |
| [Gazebo models and worlds collection](https://github.com/leonhartyao/gazebo_models_worlds_collection) | `cce115b82691b7c529a02f47e5efa391145a4ca1` | GPL-3.0 | Indoor Test Zone model used by the multi-agent simulation |
| [Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) | ROS Jazzy binary package | EPL-2.0 or BSD-3-Clause | Docker-friendly ROS middleware |
| [jsonschema](https://github.com/python-jsonschema/jsonschema) | Ubuntu 24.04 package | MIT | Draft 7 mission validation |
| [OpenAI structured outputs](https://developers.openai.com/api/docs/guides/structured-outputs) | API documentation reviewed July 2026 | Service documentation; no code copied | Schema-constrained mission proposal |
| [Gemini structured output](https://ai.google.dev/gemini-api/docs/generate-content/structured-output) | `gemini-2.5-flash` | Service documentation; no code copied | Schema-constrained mission proposal |

The simulation launch follows the composition of the Apache-2.0 TurtleBot3
Jazzy launch files, with server/client separation for headless operation. The
world file replaces network-hosted ground and light models with local SDF
elements.

Files under `src/omokai_*`, `docker/`, and `scripts/` are original
implementation unless stated above. The Test Zone source, license, mesh, and
texture are retained under `src/omokai_bringup/models/test_zone/`. Transitive
Ubuntu and ROS packages retain their respective licenses.
