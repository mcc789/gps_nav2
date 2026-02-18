from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcy',
    maintainer_email='mcy@todo.todo',
    description='TODO: Package description',
    license='Apache2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_goal_publisher = gps_nav2.gps_goal_publisher:main',
        ],
    },
)
