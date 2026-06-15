from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turret_vision'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Turret-mounted face recognition for Mercury UGVC',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'recognition = turret_vision.recognition:main',
            'scanner = turret_vision.scanner:main',
            'turret = turret_vision.turret:main',
            'trigger = turret_vision.trigger:main',
        ],
    },
)