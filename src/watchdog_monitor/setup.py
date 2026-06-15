from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'watchdog_monitor'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Monitoring and observability for Mercury robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health = watchdog_monitor.health:main',
            'waypoints = watchdog_monitor.waypoints:main',
            'dashboard = watchdog_monitor.dashboard:main',
        ],
    },
)