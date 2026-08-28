"""Publish the xacro-managed mobile-robot description and optionally open RViz."""

from math import radians
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _number(value) -> str:
    return f"{float(value):.12g}"


def _xacro_mappings(params: dict) -> dict[str, str]:
    camera_xyz = params["camera_optical_xyz_m"]
    camera_rpy = [radians(value) for value in params["camera_mount_rpy_deg"]]
    camera_size = params["camera_box_size_m"]
    chassis_size = params["chassis_frame_size_m"]
    chassis_centre = params["chassis_frame_centre_from_base_link_m"]
    mesh_origin = params["cart_frame_mesh_origin_from_base_link_m"]
    return {
        "base_link_z": _number(params["base_link_z_m"]),
        "camera_x": _number(camera_xyz[0]),
        "camera_y": _number(camera_xyz[1]),
        "camera_z": _number(camera_xyz[2]),
        "camera_roll": _number(camera_rpy[0]),
        "camera_pitch": _number(camera_rpy[1]),
        "camera_yaw": _number(camera_rpy[2]),
        "camera_depth": _number(camera_size[0]),
        "camera_width": _number(camera_size[1]),
        "camera_height": _number(camera_size[2]),
        "chassis_length": _number(chassis_size[0]),
        "chassis_width": _number(chassis_size[1]),
        "chassis_height": _number(chassis_size[2]),
        "chassis_centre_x": _number(chassis_centre[0]),
        "chassis_centre_y": _number(chassis_centre[1]),
        "chassis_centre_z": _number(chassis_centre[2]),
        "wheel_separation": _number(params["wheel_separation_m"]),
        "wheel_radius": _number(params["wheel_radius_m"]),
        "wheel_width": _number(params["wheel_width_m"]),
        "cart_mesh_x": _number(mesh_origin[0]),
        "cart_mesh_y": _number(mesh_origin[1]),
        "cart_mesh_z": _number(mesh_origin[2]),
        "mesh_scale": _number(params["mesh_scale_m_per_mm"]),
        "chassis_mass": _number(params["chassis_mass_kg"]),
        "wheel_mass": _number(params["wheel_mass_kg"]),
        "camera_mass": _number(params["camera_mass_kg"]),
    }


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mr_description"))
    params = yaml.safe_load((share / "config" / "robot_params.yaml").read_text(encoding="utf-8"))
    document = xacro.process_file(
        str(share / "urdf" / "mobile_robot.urdf.xacro"),
        mappings=_xacro_mappings(params),
    )
    robot_description = ParameterValue(document.toxml(), value_type=str)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    joint_state_gui = LaunchConfiguration("joint_state_gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            DeclareLaunchArgument("joint_state_gui", default_value="false"),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(publish_joint_states),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(joint_state_gui),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(share / "rviz" / "mobile_robot.rviz")],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
