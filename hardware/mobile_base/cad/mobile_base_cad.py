# Refactored via build123d: Fully independent parametric CAD reimplementation
# to eliminate copyright/licensing restrictions of the original 3D mesh.

"""Independent parametric CAD source for the mobile robot base.

The reference cart STEP is intentionally not imported.  This file recreates the
measured envelope and functional layout from build123d primitives so STEP is the
primary artifact and the ROS STL files remain reproducible derivatives.

Units: millimetres.
Coordinates: +X forward, +Y left, +Z up.  The CAD origin is preserved from the
measured cart assembly; BASE_LINK_IN_CAD is the ROS base_link origin.
"""

from __future__ import annotations

from math import atan2, degrees, sqrt

from build123d import (
    Align,
    Axis,
    Box,
    Circle,
    Color,
    Compound,
    Cylinder,
    Helix,
    Location,
    Plane,
    Vector,
    sweep,
)
from cadpy.assembly import AssemblyHelper


# Measured cart and robot datums.
MEASURED_TOTAL_MASS_KG = 9.8
# URDF allocation: static chassis 8.6 kg, two wheels 0.5 kg each, camera 0.2 kg.
# STEP is the geometric master and carries no assumed material-density model.
GROUND_Z = -6.517313
FRAME_MIN_Z = 82.0
FRAME_MAX_Z = 450.0
FRAME_LENGTH_X = 450.0
FRAME_WIDTH_Y = 400.0
FRAME_HEIGHT_Z = FRAME_MAX_Z - FRAME_MIN_Z
ASSEMBLY_HALF_X = 257.764801
ASSEMBLY_HALF_Y = 200.0

BASE_LINK_IN_CAD = (-25.0, 0.0, 43.482687)
WHEEL_X = BASE_LINK_IN_CAD[0]
WHEEL_Z = BASE_LINK_IN_CAD[2]
WHEEL_CENTRE_Y = 132.0
WHEEL_RADIUS = 50.0
WHEEL_WIDTH = 24.0
CASTER_WHEEL_RADIUS = 35.0
FRONT_CASTER_YAW_DEG = 0.0
REAR_CASTER_YAW_DEG = 180.0

# Independently recreated 20-series frame layout.
EXTRUSION = 20.0
FRAME_X_RAIL_Y = 190.0
FRAME_Y_RAIL_X = 215.0
LOWER_RAIL_Z = 92.0
LOWER_INNER_RAIL_Z = LOWER_RAIL_Z
UPPER_RAIL_Z = 440.0
MIDDLE_RAIL_Z = 238.0
POST_LENGTH = 328.0
POST_CENTRE_Z = 266.0

# Nominal hinge-spring module datums adapted to the 264 mm wheel contract.
FRAME_RAIL_Y = 85.0
PIVOT_X = WHEEL_X - 95.0
HINGE_PIN_Z = WHEEL_Z + 30.0
FRAME_UNDERSIDE_Z = FRAME_MIN_Z
ROCKER_TOP_Z = WHEEL_Z - 27.0
ROCKER_LENGTH = 150.0
ROCKER_WIDTH = 30.0
ROCKER_THICKNESS = 3.0
SPRING_FREE_LENGTH = 60.0
SPRING_NOMINAL_LENGTH = 50.2
SPRING_OD = 12.0
SPRING_WIRE_D = 1.4
SPRING_TURNS = 8.5


COLORS = {
    "frame": Color(0.62, 0.68, 0.78),
    "caster": Color(0.34, 0.37, 0.43),
    "caster_wheel": Color(0.08, 0.09, 0.11),
    "rocker": Color(0.70, 0.72, 0.75),
    "hinge": Color(0.12, 0.13, 0.15),
    "spring": Color(0.76, 0.79, 0.84),
    "motor": Color(0.22, 0.24, 0.28),
    "shaft": Color(0.77, 0.79, 0.82),
    "wheel": Color(0.025, 0.025, 0.03),
    "hub": Color(0.58, 0.61, 0.67),
}


def _label(shape, name: str, color_key: str):
    shape.label = name
    shape.color = COLORS[color_key]
    return shape


def _box(size_x: float, size_y: float, size_z: float, centre):
    return Box(
        size_x,
        size_y,
        size_z,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).translate(centre)


def _cylinder_z(radius: float, length: float, centre):
    return Cylinder(
        radius,
        length,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).translate(centre)


def _cylinder_y(radius: float, length: float, centre):
    return (
        Cylinder(
            radius,
            length,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        .rotate(Axis.X, 90.0)
        .translate(centre)
    )


def _t_slot_rail(axis: str, length: float, centre):
    """Create a new square extrusion envelope with four shallow centre slots."""

    cx, cy, cz = centre
    slot_width = 5.0
    slot_depth = 1.4
    surface_offset = EXTRUSION / 2.0 - slot_depth / 2.0

    if axis == "x":
        rail = _box(length, EXTRUSION, EXTRUSION, centre)
        cutters = [
            _box(length + 2.0, slot_width, slot_depth + 0.2, (cx, cy, cz + surface_offset)),
            _box(length + 2.0, slot_width, slot_depth + 0.2, (cx, cy, cz - surface_offset)),
            _box(length + 2.0, slot_depth + 0.2, slot_width, (cx, cy + surface_offset, cz)),
            _box(length + 2.0, slot_depth + 0.2, slot_width, (cx, cy - surface_offset, cz)),
        ]
    elif axis == "y":
        rail = _box(EXTRUSION, length, EXTRUSION, centre)
        cutters = [
            _box(slot_width, length + 2.0, slot_depth + 0.2, (cx, cy, cz + surface_offset)),
            _box(slot_width, length + 2.0, slot_depth + 0.2, (cx, cy, cz - surface_offset)),
            _box(slot_depth + 0.2, length + 2.0, slot_width, (cx + surface_offset, cy, cz)),
            _box(slot_depth + 0.2, length + 2.0, slot_width, (cx - surface_offset, cy, cz)),
        ]
    elif axis == "z":
        rail = _box(EXTRUSION, EXTRUSION, length, centre)
        cutters = [
            _box(slot_depth + 0.2, slot_width, length + 2.0, (cx + surface_offset, cy, cz)),
            _box(slot_depth + 0.2, slot_width, length + 2.0, (cx - surface_offset, cy, cz)),
            _box(slot_width, slot_depth + 0.2, length + 2.0, (cx, cy + surface_offset, cz)),
            _box(slot_width, slot_depth + 0.2, length + 2.0, (cx, cy - surface_offset, cz)),
        ]
    else:
        raise ValueError("axis must be x, y, or z")

    for cutter in cutters:
        rail = rail - cutter
    return rail


def make_frame() -> Compound:
    parts = []

    def add(axis: str, length: float, centre, name: str):
        parts.append(_label(_t_slot_rail(axis, length, centre), name, "frame"))

    for level, z in (("lower", LOWER_RAIL_Z), ("upper", UPPER_RAIL_Z)):
        for side, y in (("left", FRAME_X_RAIL_Y), ("right", -FRAME_X_RAIL_Y)):
            add("x", FRAME_LENGTH_X, (0.0, y, z), f"{level}_{side}_x_rail")
        for side, x in (("front", FRAME_Y_RAIL_X), ("rear", -FRAME_Y_RAIL_X)):
            add("y", FRAME_WIDTH_Y, (x, 0.0, z), f"{level}_{side}_y_rail")

    for x_name, x in (("front", FRAME_Y_RAIL_X), ("rear", -FRAME_Y_RAIL_X)):
        for y_name, y in (("left", FRAME_X_RAIL_Y), ("right", -FRAME_X_RAIL_Y)):
            add("z", POST_LENGTH, (x, y, POST_CENTRE_Z), f"{x_name}_{y_name}_post")

    for side, y in (("left", FRAME_X_RAIL_Y), ("right", -FRAME_X_RAIL_Y)):
        add("x", FRAME_LENGTH_X, (0.0, y, MIDDLE_RAIL_Z), f"middle_{side}_x_rail")
    for side, x in (("front", FRAME_Y_RAIL_X), ("rear", -FRAME_Y_RAIL_X)):
        add("y", FRAME_WIDTH_Y, (x, 0.0, MIDDLE_RAIL_Z), f"middle_{side}_y_rail")

    for index, y in enumerate((-65.0, 65.0), start=1):
        add("x", 410.0, (0.0, y, LOWER_INNER_RAIL_Z), f"lower_inner_x_rail_{index}")
    for index, x in enumerate((-70.0, 70.0), start=1):
        add("y", 360.0, (x, 0.0, LOWER_INNER_RAIL_Z), f"lower_inner_y_rail_{index}")

    return Compound(label="independent_20_series_cart_frame", children=parts)


def _diagonal_fork_arm(y: float, wheel_offset: float):
    upper_z = 66.0
    lower_z = WHEEL_Z - 15.0
    delta_z = lower_z - upper_z
    length = sqrt(wheel_offset * wheel_offset + delta_z * delta_z)
    angle_y = degrees(atan2(wheel_offset, delta_z))
    centre = (wheel_offset / 2.0, y, (upper_z + lower_z) / 2.0)
    return _box(7.0, 6.0, length, (0.0, 0.0, 0.0)).rotate(
        Axis.Y, angle_y
    ).translate(centre)


def make_caster(name: str, plate_xy, yaw_deg: float) -> Compound:
    """Create a simplified independent swivel-caster envelope."""

    wheel_offset = ASSEMBLY_HALF_X - CASTER_WHEEL_RADIUS - 190.0
    wheel_centre = (wheel_offset, 0.0, GROUND_Z + CASTER_WHEEL_RADIUS)
    children = [
        _label(_box(70.0, 70.0, 5.0, (0.0, 0.0, 79.5)), f"{name}_mount_plate", "caster"),
        _label(_cylinder_z(16.0, 11.0, (0.0, 0.0, 71.5)), f"{name}_swivel_bearing", "caster"),
        _label(_cylinder_z(7.0, 13.0, (0.0, 0.0, 61.0)), f"{name}_swivel_stem", "shaft"),
        _label(_diagonal_fork_arm(-12.0, wheel_offset), f"{name}_fork_arm_a", "caster"),
        _label(_diagonal_fork_arm(12.0, wheel_offset), f"{name}_fork_arm_b", "caster"),
        _label(_cylinder_y(CASTER_WHEEL_RADIUS, 18.0, wheel_centre), f"{name}_wheel", "caster_wheel"),
        _label(_cylinder_y(6.0, 26.0, wheel_centre), f"{name}_axle", "shaft"),
    ]
    caster = Compound(label=name, children=children).rotate(Axis.Z, yaw_deg)
    return caster.translate((plate_xy[0], plate_xy[1], 0.0))


def make_casters() -> Compound:
    casters = [
        make_caster("front_left_caster", (190.0, 165.0), FRONT_CASTER_YAW_DEG),
        make_caster("rear_right_caster", (-190.0, -165.0), REAR_CASTER_YAW_DEG),
        make_caster("rear_left_caster", (-190.0, 165.0), REAR_CASTER_YAW_DEG),
        make_caster("front_right_caster", (190.0, -165.0), FRONT_CASTER_YAW_DEG),
    ]
    return Compound(label="four_static_casters", children=casters)


def _make_vertical_spring(x: float, y: float):
    wire_radius = SPRING_WIRE_D / 2.0
    lower_z = FRAME_UNDERSIDE_Z - SPRING_NOMINAL_LENGTH
    helix_height = SPRING_NOMINAL_LENGTH - 2.0 * wire_radius
    start = Vector((x, y, lower_z + wire_radius))
    path = Helix(
        helix_height / SPRING_TURNS,
        helix_height,
        (SPRING_OD - SPRING_WIRE_D) / 2.0,
        center=start,
        direction=Vector(0.0, 0.0, 1.0),
    )
    section = Plane(origin=path.position_at(0), z_dir=path.tangent_at(0)) * Circle(wire_radius)
    return sweep(section, path)


def make_spring_drive_module(side: str) -> Compound:
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    sign = 1.0 if side == "left" else -1.0
    rail_y = sign * FRAME_RAIL_Y
    wheel_y = sign * WHEEL_CENTRE_Y
    children = []

    for index, offset in enumerate((-15.0, 15.0), start=1):
        rocker_y = rail_y + sign * offset
        rocker = _box(
            ROCKER_LENGTH,
            ROCKER_WIDTH,
            ROCKER_THICKNESS,
            (PIVOT_X - 15.0 + ROCKER_LENGTH / 2.0, rocker_y, ROCKER_TOP_Z - ROCKER_THICKNESS / 2.0),
        )
        children.append(_label(rocker, f"{side}_rocker_{index}_150x30x3", "rocker"))

    for index, offset in enumerate((-10.0, 10.0), start=1):
        hinge_y = rail_y + sign * offset
        fixed_leaf = _box(16.0, 20.0, 4.0, (PIVOT_X - 8.0, hinge_y, FRAME_UNDERSIDE_Z - 2.0))
        moving_leaf = _box(16.0, 20.0, 4.0, (PIVOT_X + 8.0, hinge_y, FRAME_UNDERSIDE_Z - 2.0))
        barrel = _cylinder_y(4.0, 20.0, (PIVOT_X, hinge_y, HINGE_PIN_Z))
        pin = _cylinder_y(2.0, 22.0, (PIVOT_X, hinge_y, HINGE_PIN_Z))
        children.extend(
            [
                _label(fixed_leaf, f"{side}_hinge_{index}_fixed_leaf", "hinge"),
                _label(moving_leaf, f"{side}_hinge_{index}_moving_leaf", "hinge"),
                _label(barrel, f"{side}_hinge_{index}_barrel", "hinge"),
                _label(pin, f"{side}_hinge_{index}_pin", "shaft"),
            ]
        )

    spring_x = PIVOT_X + 122.0
    spring_y = rail_y + sign * 10.0
    children.append(_label(_make_vertical_spring(spring_x, spring_y), f"{side}_WM12_60_spring_envelope", "spring"))
    children.extend(
        [
            _label(_cylinder_z(2.5, 70.0, (PIVOT_X + 55.0, rail_y - sign * 10.0, 49.0)), f"{side}_travel_stop", "shaft"),
            _label(_box(30.0, 3.0, 51.0, (PIVOT_X + 10.0, rail_y + sign * 28.0, 53.1)), f"{side}_hinge_link_ear", "rocker"),
            _label(_box(80.0, 60.0, 3.0, (WHEEL_X, rail_y, ROCKER_TOP_Z + 1.5)), f"{side}_motor_mount_base", "motor"),
            _label(_box(44.0, 3.0, 42.0, (WHEEL_X, rail_y + sign * 20.0, WHEEL_Z)), f"{side}_motor_mount_wall", "motor"),
        ]
    )

    face_y = rail_y + sign * 20.0
    children.extend(
        [
            _label(_cylinder_y(18.5, 22.0, (WHEEL_X, face_y - sign * 11.0, WHEEL_Z)), f"{side}_gearbox_envelope", "motor"),
            _label(_cylinder_y(17.5, 38.0, (WHEEL_X, face_y - sign * 41.0, WHEEL_Z)), f"{side}_motor_envelope", "motor"),
            _label(_cylinder_y(17.0, 13.0, (WHEEL_X, face_y - sign * 66.5, WHEEL_Z)), f"{side}_encoder_envelope", "motor"),
            _label(_cylinder_y(3.0, abs(wheel_y - face_y), (WHEEL_X, (wheel_y + face_y) / 2.0, WHEEL_Z)), f"{side}_drive_shaft", "shaft"),
        ]
    )

    module = Compound(label=f"{side}_hinge_spring_drive_module", children=children)
    return module


def make_drive_wheel():
    """Create the one generic wheel geometry reused by both wheel links."""

    tyre = Cylinder(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ) - Cylinder(
        34.0,
        WHEEL_WIDTH + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    hub = Cylinder(
        18.0,
        14.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ) - Cylinder(
        6.0,
        16.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    spoke_limit = Cylinder(34.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    spokes = None
    for angle in range(0, 360, 72):
        spoke = _box(7.0, 56.0, 8.0, (0.0, 0.0, 0.0)).rotate(Axis.Z, float(angle))
        spoke = spoke & spoke_limit
        spokes = spoke if spokes is None else spokes.fuse(spoke)
    wheel = tyre.fuse(hub).fuse(spokes)
    wheel = wheel.rotate(Axis.X, 90.0)
    return _label(wheel, "generic_drive_wheel_100x24", "wheel")


def build_static_geometry() -> Compound:
    return Compound(
        label="cart_frame_casters_and_static_drive",
        children=[
            make_frame(),
            make_casters(),
            make_spring_drive_module("left"),
            make_spring_drive_module("right"),
        ],
    )


def build_full_assembly() -> Compound:
    asm = AssemblyHelper("mobile_robot_base")
    static = asm.add(build_static_geometry(), "cart_frame_casters_and_static_drive")
    left_wheel = asm.add(make_drive_wheel(), "left_drive_wheel")
    right_wheel = asm.add(make_drive_wheel(), "right_drive_wheel")

    left_fixed = asm.revolute_frame(
        static,
        "left_wheel_axis",
        Axis((WHEEL_X, WHEEL_CENTRE_Y, WHEEL_Z), (0.0, 1.0, 0.0)),
    )
    left_moving = asm.rigid_frame(
        left_wheel,
        "left_wheel_local_axis",
        Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)),
    )
    asm.revolute(left_fixed, left_moving, angle=0.0, label="left_wheel_joint")

    right_fixed = asm.revolute_frame(
        static,
        "right_wheel_axis",
        Axis((WHEEL_X, -WHEEL_CENTRE_Y, WHEEL_Z), (0.0, 1.0, 0.0)),
    )
    right_moving = asm.rigid_frame(
        right_wheel,
        "right_wheel_local_axis",
        Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)),
    )
    asm.revolute(right_fixed, right_moving, angle=0.0, label="right_wheel_joint")
    return asm.build()


def validation_facts() -> dict:
    return {
        "units": "mm",
        "cad_origin_preserved": True,
        "base_link_in_cad_mm": list(BASE_LINK_IN_CAD),
        "mesh_origin_from_base_link_m": [0.025, 0.0, -0.043482687],
        "static_bbox_size_mm": [2.0 * ASSEMBLY_HALF_X, 2.0 * ASSEMBLY_HALF_Y, FRAME_MAX_Z - GROUND_Z],
        "wheel_size_mm": [2.0 * WHEEL_RADIUS, WHEEL_WIDTH, 2.0 * WHEEL_RADIUS],
        "wheel_centres_in_cad_mm": [
            [WHEEL_X, WHEEL_CENTRE_Y, WHEEL_Z],
            [WHEEL_X, -WHEEL_CENTRE_Y, WHEEL_Z],
        ],
        "wheel_track_mm": 2.0 * WHEEL_CENTRE_Y,
        "wheel_joint_axis": [0.0, 1.0, 0.0],
        "caster_yaw_deg": {
            "front_left": FRONT_CASTER_YAW_DEG,
            "front_right": FRONT_CASTER_YAW_DEG,
            "rear_left": REAR_CASTER_YAW_DEG,
            "rear_right": REAR_CASTER_YAW_DEG,
        },
        "caster_wheel_axis": [0.0, 1.0, 0.0],
        "caster_rolling_direction": [1.0, 0.0, 0.0],
        "spring_installed_length_mm": SPRING_NOMINAL_LENGTH,
        "spring_free_length_mm": SPRING_FREE_LENGTH,
    }


def gen_step() -> Compound:
    return build_full_assembly()


if __name__ == "__main__":
    import json

    print(json.dumps(validation_facts(), indent=2))
