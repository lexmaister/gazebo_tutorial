from setuptools import find_packages, setup

package_name = 'panda_gz_ros2_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lexmaister',
    maintainer_email='aleev@simicon.com',
    description='This example package demonstrates how to control a Panda manipulator in Gazebo Sim using ROS 2 position control (pos2_control), with topic bridging powered by ros_gz_bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
