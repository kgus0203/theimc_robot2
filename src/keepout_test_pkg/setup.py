import os
from glob import glob

from setuptools import setup


package_name = 'keepout_test_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeff@todo.com',
    description='Display-only Nav2 keepout-filter reproduction package.',
    license='Apache-2.0',
)
