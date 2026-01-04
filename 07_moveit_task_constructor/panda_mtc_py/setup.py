from setuptools import find_packages, setup
import os
from glob import glob

package_name = "panda_mtc_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files from the 'launch' and 'config' directories
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lexmaister",
    maintainer_email="lexmaister@gmail.com",
    description="Tutorial python package with an example of basic MoveIt Task Constructor's API usage for pick and place operation",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            f"mtc_node = {package_name}.mtc_node:main",
            f"mtc_pickplace_node = {package_name}.mtc_pickplace_node:main",
            f"wait_env_ready = {package_name}.wait_env_ready:main",
            f"add_scene_from_yaml = {package_name}.add_scene_from_yaml:main",
        ],
    },
)
