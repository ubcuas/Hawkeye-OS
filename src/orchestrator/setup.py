from setuptools import setup
import os

package_name = 'orchestrator'

def parse_requirements(filename):
    requirements = []
    if os.path.exists(filename):
        with open(filename, 'r') as f:
                requirements = [line.strip() for line in f
                                if line.strip() and not line.startswith('#')]
                
    return requirements

here = os.path.abspath(os.path.dirname(__file__))
requirements_path = os.path.join(here, '../../requirements.txt')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'] + parse_requirements(requirements_path),
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