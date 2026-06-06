from setuptools import setup, find_packages
from glob import glob
import os

package_name = "perception"

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
                "launch",
            ),
            glob("launch/*.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.json"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="w1dow",
    maintainer_email="w1dow@example.com",
    description="Perception stack for lane detection and costmaps",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            # old costmap node (kept but not launched by default anymore)
            'calibrate_homography = perception.calibrate_homography:main',
            # NEW: lightweight detection node that feeds lane_assist_node
            'lane_detection = perception.lane_detection:main',
            'lane_costmap = perception.lane_costmap_node:main',
            'lane_assist_node = perception.lane_assist_node:main',
            'pothole_costmap = perception.pothole_costmap_node:main',
            'follower = perception.final:main',
            'test = perception.test:main'
        ],
    },
)