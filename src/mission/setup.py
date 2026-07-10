from setuptools import find_packages, setup
from glob import glob
import os

package_name = "mission"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch"
            ),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config"
            ),
            glob("config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daksh Panchal",
    maintainer_email="dakshpanchal08@gmail.com",
    description="Mission execution and waypoint sequencing for Mercury UGV",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mission_setup = mission.mission_setup:main",
            "waypoint_sender = mission.waypoint_sender:main",
            "send_gps_goal = mission.send_gps_goal:main",
        ],
    },
)