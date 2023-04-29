from os.path import join
from glob import glob
from setuptools import setup, find_packages

package_name = 'boat_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devon Friend',
    maintainer_email='devon.friend45@gmail.com',
    description='UBC Sailbot\'s Boat Simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physics_engine_node = boat_simulator.physics_engine.physics_engine_node:main',
            'output_interface_node = boat_simulator.output_interface.output_interface_node:main'
        ],
    },
)
