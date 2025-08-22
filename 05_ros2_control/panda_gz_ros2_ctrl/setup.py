from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'panda_gz_ros2_ctrl'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # SDF files
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*')),
        # Robot descripion
        (os.path.join('share', package_name, 'panda_description', 'urdf'), glob('panda_description/urdf/*')),
        (os.path.join('share', package_name, 'panda_description', 'meshes', 'visual'), glob('panda_description/meshes/visual/*')),
        (os.path.join('share', package_name, 'panda_description', 'meshes', 'collision'), glob('panda_description/meshes/collision/*')),
        # ros2_control_config
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lexmaister',
    maintainer_email='lexmaister@gmail.com',
    description='This example package demonstrates how to control a Panda manipulator in Gazebo Sim using ROS 2 position control (pos2_control), with topic bridging powered by ros_gz_bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
