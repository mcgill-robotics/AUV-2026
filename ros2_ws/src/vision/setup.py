# SPDX-License-Identifier: GPL-3.0-or-later

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_collection = vision.image_collection:main',
            'front_cam_object_detection = vision.front_cam_object_detection:main',
            'down_cam_object_detection = vision.down_cam_object_detection:main',
            'down_image_enhancement = vision.down_image_enhancement:main',
            'front_image_enhancement = vision.front_image_enhancement:main',
        ],
    },
)
