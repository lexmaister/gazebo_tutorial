from setuptools import find_packages, setup
import os
from glob import glob


package_name = "panda_gz_moveit"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Install marker file in the package index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        # SDF files
        (os.path.join("share", package_name, "sdf"), glob("sdf/*")),
        # Robot descripion
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        # moveit and ros2_control config files
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lexmaister",
    maintainer_email="lexmaister@gmail.com",
    description="This example package demonstrates how to privide motion planning for a Panda manipulator with MoveIt and Gazebo Sim.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["demo_control = panda_gz_moveit.demo_control:main"],
    },
)
