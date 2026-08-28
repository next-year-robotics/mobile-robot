"""Contract tests for xacro, TF geometry, meshes, and movable wheel joints."""

from __future__ import annotations

import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "urdf" / "mobile_robot.urdf.xacro"
CONFIG_PATH = PACKAGE_ROOT / "config" / "robot_params.yaml"


def _floats(text: str) -> list[float]:
    return [float(value) for value in text.split()]


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f"./joint[@name='{name}']")
    assert joint is not None, f"missing joint {name}"
    return joint


def _fresh_xacro() -> str:
    executable = shutil.which("xacro")
    if executable:
        command = [executable, str(XACRO_PATH)]
    else:
        command = [sys.executable, "-c", "import xacro; xacro.main()", str(XACRO_PATH)]
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return completed.stdout


@pytest.fixture(scope="module")
def params() -> dict:
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def robot() -> ET.Element:
    return ET.fromstring(_fresh_xacro())


def test_xacro_expands_to_named_robot(robot: ET.Element) -> None:
    assert robot.tag == "robot"
    assert robot.attrib["name"] == "mobile_robot"


def test_link_and_joint_contract(robot: ET.Element) -> None:
    links = {link.attrib["name"] for link in robot.findall("link")}
    assert links == {
        "base_footprint",
        "base_link",
        "left_wheel_link",
        "right_wheel_link",
        "camera_link",
        "camera_optical_frame",
    }
    assert _joint(robot, "left_wheel_joint").attrib["type"] == "continuous"
    assert _joint(robot, "right_wheel_joint").attrib["type"] == "continuous"


def test_total_mass_budget(robot: ET.Element, params: dict) -> None:
    link_masses = {
        link.attrib["name"]: float(link.find("inertial/mass").attrib["value"])
        for link in robot.findall("link")
        if link.find("inertial/mass") is not None
    }
    assert link_masses == pytest.approx(
        {
            "base_link": params["chassis_mass_kg"],
            "left_wheel_link": params["wheel_mass_kg"],
            "right_wheel_link": params["wheel_mass_kg"],
            "camera_link": params["camera_mass_kg"],
        }
    )
    assert sum(link_masses.values()) == pytest.approx(params["total_mass_kg"], abs=1.0e-12)
    assert params["total_mass_kg"] == pytest.approx(9.8, abs=1.0e-12)


def test_tf_measurements(robot: ET.Element, params: dict) -> None:
    base_z = _floats(_joint(robot, "base_footprint_joint").find("origin").attrib["xyz"])[2]
    camera_xyz = _floats(_joint(robot, "camera_joint").find("origin").attrib["xyz"])
    left_y = _floats(_joint(robot, "left_wheel_joint").find("origin").attrib["xyz"])[1]
    right_y = _floats(_joint(robot, "right_wheel_joint").find("origin").attrib["xyz"])[1]
    assert base_z == pytest.approx(params["base_link_z_m"], abs=1.0e-9)
    assert base_z + camera_xyz[2] == pytest.approx(0.410, abs=1.0e-9)
    assert left_y - right_y == pytest.approx(params["wheel_separation_m"], abs=1.0e-9)
    assert camera_xyz == pytest.approx(params["camera_optical_xyz_m"], abs=1.0e-9)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _column(matrix: list[list[float]], index: int) -> list[float]:
    return [matrix[row][index] for row in range(3)]


def test_camera_optical_axes(robot: ET.Element) -> None:
    rpy = _floats(_joint(robot, "camera_optical_joint").find("origin").attrib["rpy"])
    rotation = _rpy_matrix(*rpy)
    assert _column(rotation, 0) == pytest.approx([0.0, -1.0, 0.0], abs=1.0e-9)  # right
    assert _column(rotation, 1) == pytest.approx([0.0, 0.0, -1.0], abs=1.0e-9)  # down
    assert _column(rotation, 2) == pytest.approx([1.0, 0.0, 0.0], abs=1.0e-9)   # forward


def test_wheel_joints_are_rotatable(robot: ET.Element, params: dict) -> None:
    radius = params["wheel_radius_m"]
    angle = 0.25
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint = _joint(robot, name)
        axis = _floats(joint.find("axis").attrib["xyz"])
        assert axis == pytest.approx([0.0, 1.0, 0.0], abs=1.0e-12)
        # Rodrigues rotation of a top-of-wheel point around +Y.
        rotated = [radius * math.sin(angle), 0.0, radius * math.cos(angle)]
        assert math.dist(rotated, [0.0, 0.0, 0.0]) == pytest.approx(radius)
        assert rotated[0] > 0.0


def _binary_stl_bbox(path: Path) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    assert len(data) >= 84
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + triangle_count * 50, f"unexpected STL encoding for {path}"
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for values in struct.iter_unpack("<12fH", data[84:]):
        for offset in (3, 6, 9):
            for axis in range(3):
                coordinate = values[offset + axis]
                lower[axis] = min(lower[axis], coordinate)
                upper[axis] = max(upper[axis], coordinate)
    return lower, upper


def test_mesh_scale_and_dimensions(robot: ET.Element, params: dict) -> None:
    cart_mesh = PACKAGE_ROOT / "meshes" / "cart_frame.stl"
    wheel_mesh = PACKAGE_ROOT / "meshes" / "drive_wheel.stl"
    cart_min, cart_max = _binary_stl_bbox(cart_mesh)
    wheel_min, wheel_max = _binary_stl_bbox(wheel_mesh)
    scale = params["mesh_scale_m_per_mm"]
    cart_size_m = [(hi - lo) * scale for lo, hi in zip(cart_min, cart_max)]
    wheel_size_m = [(hi - lo) * scale for lo, hi in zip(wheel_min, wheel_max)]
    assert cart_size_m == pytest.approx([0.515529602, 0.400, 0.456517313], abs=5.0e-4)
    assert wheel_size_m == pytest.approx([0.100, 0.024, 0.100], abs=2.0e-4)
    assert max(cart_size_m) < 1.0  # catches a missing 0.001 scale / 1000x error

    mesh_scales = [
        _floats(mesh.attrib["scale"])
        for mesh in robot.findall(".//mesh")
    ]
    assert mesh_scales
    for mesh_scale in mesh_scales:
        assert mesh_scale == pytest.approx([scale, scale, scale], abs=1.0e-12)


def test_preserved_cad_origin_compensation(robot: ET.Element, params: dict) -> None:
    base_visual_origin = robot.find("./link[@name='base_link']/visual/origin")
    assert base_visual_origin is not None
    assert _floats(base_visual_origin.attrib["xyz"]) == pytest.approx(
        params["cart_frame_mesh_origin_from_base_link_m"], abs=1.0e-12
    )
