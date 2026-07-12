from setuptools import find_packages, setup
from glob import glob

package_name = 'actuators'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='douglas',
    maintainer_email='douglas@todo.todo',
    description='Actuator interfaces and launch files for the AUV',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'actuator_interface = actuators.actuator_interface:main',
        ],
    },
)
