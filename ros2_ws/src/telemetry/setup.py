from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'telemetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Install foxglove layout files
        # (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sohaib Kaidal',
    maintainer_email='kaidalisohaib@gmail.com',
    description='Foxglove Studio layouts and telemetry configuration for AUV monitoring',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'drytest_foxglove = telemetry.drytest_foxglove:main',
        ],
    },
)
