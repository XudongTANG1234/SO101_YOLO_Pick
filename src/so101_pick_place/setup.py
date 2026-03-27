import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so101_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sst',
    maintainer_email='sst@todo.todo',
    description='YOLO + RealSense perception and MoveIt2 pick for SO-101',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detect_and_locate = so101_pick_place.detect_and_locate:main',
            'detect_and_locate_segment = so101_pick_place.detect_and_locate_segment:main',
            'pick_place = so101_pick_place.pick_place_node:main',
        ],
    },
)
