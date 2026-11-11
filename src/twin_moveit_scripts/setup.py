import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'twin_moveit_scripts'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khoi',
    maintainer_email='pmkhoi@example.com',
    description='Perception, tracking, interception, condition routing and MoveIt 2 motion for the Panda parcel cell.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detector_parcel_cell = twin_moveit_scripts.object_detector_parcel_cell:main',
            'parcel_task = twin_moveit_scripts.parcel_task:main',
            'parcel_feed_guard = twin_moveit_scripts.parcel_feed:main',
            'move_to = twin_moveit_scripts.move_to:main',
        ],
    },
)
