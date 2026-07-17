# Architecture

## Design goal

The system separates language interpretation from robot control. Model output
is treated as untrusted data and must pass two local gates before motion:

```text
operator
   |
   v
planner (Gemini, OpenAI, or deterministic fake)
   |
   v
mission JSON
   |
   +--> JSON Schema validation
   +--> route and safety policy validation
   |
   v
route compiler --> immutable execution plan
   |
   v
deterministic executor --> Nav2 NavigateToPose --> TurtleBot3 in Gazebo
```

The same accepted mission and route catalog always produce the same ordered
goals. The planner has no ROS dependency and is never called after execution
starts.

## Planning and validation

The planner returns a structured proposal containing:

- a known route identifier;
- one or more ordered traversal segments;
- a bounded speed;
- whether the robot should return home.

Draft 7 JSON Schema validation rejects malformed or unexpected fields.
Semantic validation then checks the route, direction, speed, traversal count,
and action against local policy. Stable error codes make rejections auditable.

Coordinates are never accepted from the model. They live in the packaged route
catalog and are resolved only after validation.

## Execution

The compiler converts a validated mission into immutable goals. The executor is
an event-driven finite-state machine that owns:

- goal order;
- deadlines and retry limits;
- cancellation;
- success and failure states;
- append-only execution evidence.

The ROS-specific adapter implements a narrow navigation interface using
Nav2 `NavigateToPose`. Domain logic is tested without ROS through an in-memory
adapter.

## Simulation

The Docker image contains ROS 2 Jazzy, Gazebo Harmonic, Navigation2, and
TurtleBot3 Waffle Pi packages. The launch sequence starts:

1. the owned Gazebo world and TurtleBot3 model;
2. robot state publication and sensor bridges;
3. the known map, AMCL, and Nav2;
4. an initial-pose publisher;
5. the operator-selected mission.

The world defines its own ground plane and lighting, so simulator startup does
not depend on downloading Gazebo Fuel models.

## Evidence and failure handling

Each mission directory contains the proposal, validation result, accepted
mission, compiled plan, execution events, and final result when applicable.
Files are written atomically, while execution events use append-only JSONL.

Provider errors, refusals, malformed responses, unknown routes, unsafe values,
navigation failures, timeouts, and cancellation all fail closed with an
explicit terminal state. A rejected proposal cannot reach the Nav2 adapter.
