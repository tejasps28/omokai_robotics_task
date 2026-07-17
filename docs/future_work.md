# Approach to the Additional Challenges

The core pipeline is deliberately reusable: a model proposes bounded intent,
local policy validates it, and deterministic components perform navigation or
control. The same boundary applies to the additional challenges.

## SLAM and autonomous navigation

SLAM Toolbox would consume LiDAR, odometry, and transforms to build and save a
2D map and pose graph. Mapping and localization would be separate operating
modes:

- mapping mode builds the map and selects safe frontiers through a
  deterministic exploration node;
- navigation mode reloads the map, localizes with Nav2, and accepts named
  destinations or locally validated exploration regions.

Acceptance would include map creation from an unknown world, serialization,
reload, loop-closure behavior, and obstacle-aware navigation to named goals.

## Vision target detection and following

The perception path would separate detector, tracker, target localizer,
snapshot notification, and follower. The operator would configure the target
class and confidence threshold through the mission schema.

On first confirmed detection, the system would store a timestamped image for
the operator. Depth or LiDAR association would estimate the target in the map
frame. A deterministic follower—not the LLM—would continuously send bounded
standoff goals to Nav2. Target loss, maximum pursuit time/distance, map bounds,
and emergency stop would remain local safety constraints.

## Multi-robot coordination

Three namespaced robots would run independent Nav2 stacks. A centralized
coordinator would convert validated squad intent into per-robot plans:

- split a route by path length;
- maintain line or wedge offsets in the map frame;
- enforce minimum separation;
- synchronize at mission barriers;
- regroup at explicit poses;
- degrade safely if one robot fails.

The model would choose only squad-level intent. Assignment, formation geometry,
collision avoidance, and recovery would be deterministic and auditable.

## Scaling to hardware

A physical deployment would add hardware-specific navigation adapters,
calibrated sensor models, signed configuration, authentication and authorization,
geofencing, independent emergency stop, telemetry monitoring, replayable audit
storage, and staged simulation/hardware-in-the-loop validation. Hosted model
failure would never remove the local safety or control boundary.
