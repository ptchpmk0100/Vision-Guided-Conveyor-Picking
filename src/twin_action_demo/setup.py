import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'twin_action_demo'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # For the YAML config file
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # S20 PERCEPTION: install the sensors-enabled world so the launch file
        # can point gz_args at share/twin_action_demo/worlds/sensors_empty.sdf.
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # S22: install the graspable cube model so panda_bringup can spawn it as
        # part of the world (one launch brings up robot + camera + cube).
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pmkhoi',
    maintainer_email='pmkhoi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_joint_server = twin_action_demo.move_joint_server:main',
            'move_joint_client = twin_action_demo.move_joint_client:main',
            'move_arm_server = twin_action_demo.move_arm_server:main',
            'finger_state_publisher = twin_action_demo.finger_state_publisher:main',
        ],
    },
)
