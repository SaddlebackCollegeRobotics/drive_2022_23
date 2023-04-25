import glob
import os
from setuptools import setup

package_name = 'driver'

setup(
    name=package_name,
    version='69.420.80085',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include launch file. TODO add launch file
        #('share/' + package_name, ['launch/<filename>.launch.py']),
        # Include gamepad config file
        ('share/' + package_name, ['driver/gamepads.config']),
        # Include kinematics math file
        ('share/' + package_name, ['driver/diff_drive_kinematics.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasper',
    maintainer_email='49565505+wluxie@users.noreply.github.com',
    description='Does something with the rover.',
    license='I AM MY OWN LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_pub = driver.controller_pub:main',
            'gamepad_diff_drive = driver.gamepad_diff_drive:main',
            'gamepad_tank_drive = driver.gamepad_tank_drive:main',
            'drive_receiver_sub = driver.drive_receiver_sub:main',
        ],
    },
)
