from setuptools import setup, find_packages

package_name = 'propulsion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/propulsion.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='McGill Robotics',
    maintainer_email='dev@mcgillrobotics.com',
    description='AUV propulsion nodes (ROS 2, Python).',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            # ros2 run propulsion thrust_mapper
            'thrust_mapper = propulsion.thrust_mapper:main',
        ],
    },
)
