import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'Catalyst'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/Catalyst.urdf.xml']),
        ('share/' + package_name + '/urdf', ['urdf/camera.urdf.xml']),
        ('share/' + package_name + '/urdf', ['urdf/camera.sdf']),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*')),
        ('share/' + package_name + '/urdf', ['urdf/Catalyst.rviz']),
        ('share/' + package_name + '/config', ['config/Catalyst_controller.yaml']),
        ('share/' + package_name + '/moveit_config', glob('Catalyst_moveit_config/config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='el7agadel',
    maintainer_email='adel0800@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = Catalyst.state_publisher:main',
            'send_trajectory = Catalyst.send_trajectory:main',
            'send_trajectory_moveit = Catalyst.send_trajectory_moveit:main',
            'camera_viewer = Catalyst.camera_viewer:main',
        ],
    },
)
