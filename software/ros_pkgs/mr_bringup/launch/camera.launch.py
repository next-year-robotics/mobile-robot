"""C920만 띄운다. M7 검증용.

usb_cam 0.8.1의 MJPEG 줄무늬 여부를 여기서 가른다.

    ros2 launch mr_bringup camera.launch.py
    ros2 run rqt_image_view rqt_image_view          # 줄무늬 없는지 확인
    ros2 topic hz /camera/image_raw                 # 25 Hz 이상
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import build_camera


def _setup(context, *args, **kwargs):
    del args, kwargs
    return build_camera(
        video_device=LaunchConfiguration('video_device').perform(context))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='',
            description='비우면 c920.yaml의 값을 쓴다.'),
        OpaqueFunction(function=_setup),
    ])
