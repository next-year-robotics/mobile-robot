from setuptools import find_packages, setup

package_name = 'mr_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/base_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emjdp',
    maintainer_email='seongho06@khu.ac.kr',
    description='휠 오도메트리와 odom -> base_footprint TF.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_node = mr_base.odom_node:main',
        ],
    },
)
