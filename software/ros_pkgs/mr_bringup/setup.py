from glob import glob

from setuptools import find_packages, setup

package_name = 'mr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emjdp',
    maintainer_email='seongho06@khu.ac.kr',
    description='launch 파일과 카메라·twist_mux 설정.',
    license='Apache-2.0',
)
