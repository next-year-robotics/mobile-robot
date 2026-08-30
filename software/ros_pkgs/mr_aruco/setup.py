from setuptools import find_packages, setup

package_name = 'mr_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/aruco_tracker.yaml',
            'config/marker_track.yaml',
            'config/follow.yaml',
            'config/align.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='emjdp',
    maintainer_email='seongho06@khu.ac.kr',
    description='ArUco 마커 추적 필터와 작업자 추종·고정 마커 정렬 제어.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'marker_track_node = mr_aruco.marker_track_node:main',
            'follow_node = mr_aruco.follow_node:main',
            'align_node = mr_aruco.align_node:main',
        ],
    },
)
