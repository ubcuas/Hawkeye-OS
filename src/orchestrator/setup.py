from setuptools import setup

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'websockets>=10.0',  # For GCOM communication
        'opencv-python',  # For reading test images
        'aiortc',  # For WebRTC streaming
        'av',  # For video frames (required by aiortc)
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Orchestrator node for async message handling with GCOM WebSocket support',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator = orchestrator.orchestrator:main',
            'mock_object_detection = orchestrator.mock_object_detection:main',
        ],
    },
)