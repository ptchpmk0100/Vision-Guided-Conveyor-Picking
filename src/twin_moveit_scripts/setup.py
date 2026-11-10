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
    description='Scripts that drive the Panda arm via moveit_py.',
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
            'llm_move = twin_moveit_scripts.llm_move:main',
            'object_detector = twin_moveit_scripts.object_detector:main',
            # A2c: multi-object HSV detector. ADDITIVE -- the single-blue node above
            # stays runnable so the verified pipeline is never left without a detector.
            'object_detector_multi = twin_moveit_scripts.object_detector_multi:main',
            # A4: sort every staged object into the bin matching its colour.
            'sort_task = twin_moveit_scripts.sort_task:main',
            # Stage B: belt-height sorting-line detector (raised scene recalibration).
            'object_detector_sorting_line = twin_moveit_scripts.object_detector_sorting_line:main',
            # Stage C/D: sort belt-height colour boxes into matching trays.
            'sort_task_sorting_line = twin_moveit_scripts.sort_task_sorting_line:main',
            # B3: dependency-free behaviour-tree orchestration of the same sort.
            'sort_tree_sorting_line = twin_moveit_scripts.sort_tree_sorting_line:main',
            # B4: independent FR-2/FR-3/FR-8 runtime invariant monitor.
            'sorting_validator = twin_moveit_scripts.sorting_validator:main',
        ],
    },
)
