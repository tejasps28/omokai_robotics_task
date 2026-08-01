# White-coat actor scenario wrappers

These SDF wrappers reference the `DoctorFemaleWalk` skin installed from
`blackcoffeerobotics/gazebo-ros-actor-plugin` at pinned revision
`e170a60b4732b0667b90b171dacb19e73208ccfd` (Apache-2.0).

- `moving.sdf` uses the cited Black Coffee path-command plugin.
- `stationary.sdf` deliberately omits both animation and motion control so
  the same person asset is frozen for the stationary-detection regression.

The mesh and textures are not duplicated here; they are supplied by the
pinned dependency built into the Docker image.
