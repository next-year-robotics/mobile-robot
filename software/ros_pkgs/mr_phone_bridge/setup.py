from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mr_phone_bridge'
here = os.path.dirname(os.path.abspath(__file__))


def contract_files():
    """계약 스키마·fixture를 share/에 복사한다. 설치본(RPi5)은 저장소 경로 없이 이것을 읽는다.

    저장소: software/ros_pkgs/mr_phone_bridge → ../../contracts
    RPi5:   ~/ros2_ws/src/mr_phone_bridge    → ../contracts (software/contracts를 src/contracts로 rsync)
    """
    for rel in ('../../contracts', '../contracts'):
        base = os.path.normpath(os.path.join(here, rel, 'phone-bridge', 'v1'))
        if os.path.isfile(os.path.join(base, 'schema', 'messages.schema.json')):
            dest = 'share/' + package_name + '/contracts/phone-bridge/v1'
            return [
                (dest + '/schema', [os.path.relpath(p, here) for p in sorted(glob(base + '/schema/*.json'))]),
                (dest + '/fixtures', [os.path.relpath(p, here) for p in sorted(glob(base + '/fixtures/*.json'))]),
            ]
    raise SystemExit('mr_phone_bridge: software/contracts/phone-bridge/v1 not found next to the package')


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/avahi', glob('config/avahi/*.service')),
        ('share/' + package_name + '/config/systemd', glob('config/systemd/*')),
    ] + contract_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emjdp',
    maintainer_email='seongho06@khu.ac.kr',
    description='phone-bridge/1 WebSocket gateway + mock bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phone_bridge = mr_phone_bridge.ros.node:main',
            'mock_bridge = mr_phone_bridge.mock.__main__:main',
        ],
    },
)
