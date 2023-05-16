"""ROS2 setup file for Drive package"""

from setuptools import setup

PKG_NAME = 'drive'

setup(
    name=PKG_NAME,
    version='69.420.80085',
    packages=[PKG_NAME],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + PKG_NAME]),
        # Include package.xml
        ('share/' + PKG_NAME, ['package.xml']),
        # Include laulaunchnch files
        ('share/' + PKG_NAME, ['launch/base_drive.launch.py']),
        ('share/' + PKG_NAME, ['launch/rover_drive.launch.py']),
        # Include gamepad config file
        ('share/' + PKG_NAME, ['config/gamepads.config']),
        # Include Diffential Drive Kinematics script
        ('share/' + PKG_NAME, [f'{PKG_NAME}/diff_drive_kinematics.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCRobotics',
    maintainer_email='49565505+wluxie@users.noreply.github.com',
    description='Drives the rover.',
    license='Apache Lisence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'gamepad_diff_drive = {PKG_NAME}.gamepad_diff_drive:main',
            f'gamepad_tank_drive = {PKG_NAME}.gamepad_tank_drive:main',
            f'motor_driver = {PKG_NAME}.odrive_motor_driver:main',
            f'diff_drive_conversion = {PKG_NAME}.diff_drive_conversion:main',
            f'calibration = {PKG_NAME}.calibration_node:main',
        ],
    },
)
