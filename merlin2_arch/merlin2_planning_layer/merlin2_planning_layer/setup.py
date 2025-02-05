from setuptools import setup
from glob import glob
import os

package_name = "merlin2_planning_layer"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="MERLIN2 Planning Layer",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
