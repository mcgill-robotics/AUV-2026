from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sohaib Kaidal',
    maintainer_email='kaidalisohaib@gmail.com',
    description='Manual teleoperation for AUV via gamepad',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_converter = teleop.teleop_converter:main',
        ],
    },
)
