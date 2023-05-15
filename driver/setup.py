from setuptools import setup

package_name = 'driver'

setup(
    name=package_name,
    version='69.420.80085',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'drive_receiver_sub = driver.drive_receiver_sub:main',
        ],
    },
)
