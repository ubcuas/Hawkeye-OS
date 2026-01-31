from setuptools import setup

package_name = 'streaming'

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
        'python-socketio[asyncio_client]>=5.0',  # For signaling server communication
        'aiohttp>=3.9.0',  # Required by python-socketio async client
        'aiortc',  # For WebRTC
        'av',  # For video frames (required by aiortc)
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Streaming node for WebRTC-based video streaming to GCOM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streaming = streaming.streaming:main',
        ],
    },
)
