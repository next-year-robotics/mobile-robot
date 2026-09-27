from glob import glob

from setuptools import find_packages, setup

package_name = 'mr_care_telemetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/systemd', glob('config/systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emjdp',
    maintainer_email='seongho06@khu.ac.kr',
    description='care-telemetry/1 Firebase RTDB agent',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'care_telemetry = mr_care_telemetry.node:main',
            'care_telemetry_sim = mr_care_telemetry.__main__:main',
        ],
    },
)
