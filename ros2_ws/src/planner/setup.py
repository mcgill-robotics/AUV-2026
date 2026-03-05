from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'planner'

setup(
    name="planner",
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + "planner"]),
        ('share/' + "planner" , ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 
					  'py_trees',
					  'py_trees_ros'],
    zip_safe=True,
    maintainer='willz',
    maintainer_email='williamzhang2205@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node = planner.RootTree:main',
            'yaw_behaviour_tree = planner.rotationLeafs.BehaviourTreeYaw:main'
        ],
    },
)
