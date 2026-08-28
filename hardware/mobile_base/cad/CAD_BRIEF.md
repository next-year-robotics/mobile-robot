# Mobile-base CAD brief

- Model: independently authored mobile-base assembly with an aluminium-extrusion
  cart, four simplified swivel casters, two nominal hinge-spring drive modules,
  and two reusable drive-wheel occurrences.
- Task type: new assembly and secondary RViz mesh exports.
- Inputs: `../cad_measurements.yaml`, `../robot_params.yaml`, the measured envelope
  of `C:/cad_work/cart/02_assembly/cart_sim.STEP`, and the geometry/interface
  parameters in
  `C:/cad_work/cart/hinge_spring_A_2026-07-31/hinge_spring_A_core.py`.
- Copyright boundary: the reference STEP is measured and visually inspected but
  never imported by this generator. All exported BREP and mesh geometry is built
  from fresh build123d primitives. Purchased-item detail is represented by new
  envelopes rather than copied supplier geometry.
- Units: millimetres in CAD and STL; URDF applies a `0.001` mesh scale.
- Mass: measured complete-robot mass is `9.8 kg`. The STEP remains a geometric
  master without artificial material densities; URDF link inertials allocate
  `8.6 kg` to the static chassis, `0.5 kg` to each drive wheel, and `0.2 kg` to
  the camera so the modeled total is exactly `9.8 kg`.
- Coordinate convention: preserve the measured STEP coordinate system. CAD
  `+x/+y/+z` equals robot forward/left/up. The CAD origin remains the aluminium
  frame plan origin. `base_link` is at `[-25, 0, 43.482687] mm` in CAD.
- Overall aligned-caster envelope: `515.529602 x 400 x 456.517313 mm`, with
  `z=[-6.517313, 450] mm`.
- Frame envelope: `450 x 400 x 368 mm`, with `z=[82, 450] mm`.
- Lower frame plane: the four internal `井`-pattern rails and the four outer
  lower perimeter rails share centre `z=92 mm`, bottom `z=82 mm`, and top
  `z=102 mm`; no internal rail is raised above the perimeter plane.
- Caster alignment: all four caster-wheel rolling planes are parallel to the
  powered drive wheels and robot `+X`; front caster yaws are `0 deg` and rear
  caster yaws are `180 deg` for a symmetric fore/aft pose.
- Drive geometry: wheel radius `50 mm`, width `24 mm`, and centres at
  `[-25, +132, 43.482687] mm` and `[-25, -132, 43.482687] mm`.
- Spring-module intent: keep the nominal 150 x 30 x 3 mm rocker, 95 mm
  pivot-to-wheel X distance, 50.2 mm installed spring length, and simplified
  hinge, motor, shaft, adapter, spring-seat, and travel-stop envelopes. The
  lateral shaft stack is adapted to the authoritative 264 mm wheel track.
- Positioning/mating: the cart is the fixed assembly root. Each wheel is placed
  with a source-level revolute mate about global `+Y`; the relation labels match
  `left_wheel_joint` and `right_wheel_joint`.
- Paths: `mobile_base_cad.py` -> `mobile_base_cad.step`;
  `cart_frame_mesh.py` -> `cart_frame_mesh.step` and the package STL;
  `drive_wheel_mesh.py` -> `drive_wheel_mesh.step` and the reusable package STL.
- Validation targets: labeled assembly, exact outer/static and wheel bounding
  boxes, 264 mm wheel-centre spacing, source-level revolute relationships,
  positive-volume solids, saved multi-view snapshots, and metre-scaled URDF
  mesh dimensions without a 1000x error.
- Assumptions: caster and purchased drive hardware are geometry envelopes, not
  manufacturing replicas. The C920 housing is omitted from CAD; URDF uses the
  permitted box envelope at the measured optical-centre transform.
