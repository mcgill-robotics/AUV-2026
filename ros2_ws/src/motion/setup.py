from setuptools import setup
import os
from glob import glob

package_name = 'motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sohaib Kaidali',
    maintainer_email='kaidalisohaib@gmail.com',
    description='AUV motion action server - orchestrates PID controllers for goal-driven navigation',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_server = motion.navigation_server:main',
        ],
    },
)
