from setuptools import find_packages, setup

package_name = 'debug_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhk-laptop',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_inspection_upload = debug_pkg.test_inspection_upload:main',
            'yolov8_visualizer_node = debug_pkg.yolov8_visualizer_node:main',	
            'check_odom_straight_node = debug_pkg.check_odom_straight:main',
            'ekf_test_cmd_sequence = debug_pkg.ekf_test_cmd_sequence:main',
        ],
    },
)
