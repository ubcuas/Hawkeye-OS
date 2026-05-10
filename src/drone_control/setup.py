from setuptools import setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'msg'), glob(os.path.join('msg', '*.msg'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='drone control node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control = drone_control.drone_control:main',
            'vslam_control = drone_control.vslam_control:main',
            # 'test_servo = drone_control.test_servo:test_servo',
            # 'test_nav = drone_control.test_nav:main',
            # 'bridge_detection = drone_control.bridge_detection:main',
            # 'test_tagged_image_pub = drone_control.test_tagged_image_pub:main',
            # 'test_bridge_pipeline = drone_control.test_bridge_pipeline:main',
            # 'test_bridge_edge_cases = drone_control.test_bridge_edge_cases:main',
            # 'test_drone_control_edges = drone_control.test_drone_control_edges:main',
        ],
    },
)