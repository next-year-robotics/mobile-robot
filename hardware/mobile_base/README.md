# Mobile-base geometry handoff

This directory is the hardware-side source of truth for the mobile-base frame,
drive-wheel, and camera placement measurements.

## Files

- `robot_params.yaml` is the compact parameter set intended for
  `software/ros_pkgs/mr_description/config/robot_params.yaml`.
- `cad_measurements.yaml` preserves the CAD origin, axis mapping, raw bounding
  boxes, derived transforms, measurement provenance, and deferred outputs.

The `mr_description` ROS package did not exist in this repository when this
handoff was created, so the integration-ready parameters are kept here until
that package is added.

## Coordinate contract

`base_footprint` is the floor point directly below the drive-axle midpoint.
Robot `+x` is forward, `+y` is left, and `+z` is up. `base_link` is 50 mm
directly above `base_footprint`, at the drive-axle height.

The confirmed transforms are:

```text
base_footprint -> base_link:      [0.000,  0.000, 0.050] m
base_link -> camera optical:      [0.250,  0.000, 0.360] m
base_link -> left wheel centre:   [0.000, +0.132, 0.000] m
base_link -> right wheel centre:  [0.000, -0.132, 0.000] m
camera mount RPY:                 [0.0, 0.0, 0.0] deg
```

The camera optical centre is therefore 410 mm above the floor and 250 mm in
front of the drive axle. It is centred laterally and looks straight forward.

## Derived-value checks

```text
camera z from base_link = 0.410 - 0.050 = 0.360 m
half wheel separation   = 0.264 / 2       = 0.132 m
wheel side inset        = 0.400 / 2 - 0.132 = 0.068 m
```

The measured “70 mm from each side” description is treated as rounded. The
existing 264 mm nominal wheel-separation contract is authoritative.

## CAD scope

The inspected STEP currently contains the aluminium frame, caster mounting
plates, and four casters. It does not yet contain the two driven wheels,
motors, C920, or camera mount. Its aluminium-frame bounding dimensions are
450 x 400 x 368 mm; these values are used only for the RViz placeholder shape.

No mount-face offset is needed for TF because the lens optical centre was
provided directly. Generate visual meshes only after the missing hardware is
added to the final CAD. A SolidWorks URDF export can then be used to
cross-check joint origins, but it should not replace the frame and link-name
contract recorded here.

## Odometry values are deliberately separate

`robot_params.yaml` contains the nominal geometry (`0.050 m` radius and
`0.264 m` separation). Do not overwrite the calibrated effective values in
`software/ros_pkgs/mr_base/config/base_params.yaml`; those values serve wheel
odometry and are expected to differ from the nominal hardware geometry.
