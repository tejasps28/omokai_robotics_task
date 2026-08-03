# DoctorFemaleWalkRed source and licence

- Source: `blackcoffeerobotics/gazebo-ros-actor-plugin`
- Revision: `e170a60b4732b0667b90b171dacb19e73208ccfd`
- Upstream skin: `config/skins/DoctorFemaleWalk`
- Licence: Apache-2.0

The Collada skin is the pinned upstream DoctorFemaleWalk mesh with its diffuse
texture reference renamed to avoid Gazebo renderer-cache collisions. The
diffuse atlas is a local coat-colour variant used to make the user-selectable
red/white acceptance scenario deterministic. Model placement is supplied by
the Omokai launch file; no simulator actor pose is consumed by perception or
following.
