from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'theimc_bringup'

setup(
    name=package_name,
    version='7.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='jeff',
    author_email='jeff@todo.com',
    maintainer='erail',
    maintainer_email='jeff@todo.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 Humble Packages for Slampibot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'bringup_node = theimc_bringup.theimc_bringup:main',
            'bringup_node = theimc_bringup.new_theimc_bringup:main',
            'robot_agent_node = theimc_bringup.robot_agent_node:main',
            'last_pose_manager_node = theimc_bringup.last_pose_manager_node:main',
            'isaac_vslam_odom_adapter_node = theimc_bringup.isaac_vslam_odom_adapter_node:main',
            'rtabmap_pose_validator_node = theimc_bringup.rtabmap_pose_validator_node:main',
            
        ],
    },
)