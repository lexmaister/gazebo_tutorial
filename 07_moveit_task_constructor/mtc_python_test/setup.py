from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mtc_python_test"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files from the 'launch' directory
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lexmaister",
    maintainer_email="lexmaister@gmail.com",
    description="Tutorial package with basic MoveIt Task Constructor usage in pick and place operation",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["mtc_node = mtc_python_test.mtc_node:main"],
    },
)
