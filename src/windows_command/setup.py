from setuptools import setup
from glob import glob
import os

package_name = 'windows_command'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Orlando E. gordillo',
    maintainer_email='ogordillo@miners.utep.edu',
    description='ROS2 node to command the Hexpod robot via windows',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_window = windows_command.main_window:main',
        ],
    },
)