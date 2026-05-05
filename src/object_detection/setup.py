from setuptools import setup

package_name = 'object_detection'

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
        # YOLO / vision
        'ultralytics==8.4.14',
        'ultralytics-thop==2.0.18',
        'clip @ git+https://github.com/ultralytics/CLIP.git@88ade288431a46233f1556d1e141901b3ef0a36b',
        # PyTorch
        'torch==2.10.0',
        'torchvision==0.25.0',
        # Image / CV
        'opencv-python==4.13.0.92',
        'pillow==12.2.0',
        # Numerics / science
        'numpy==1.26.4',
        'scipy==1.15.3',
        'matplotlib==3.10.8',
        'sympy==1.14.0',
        'mpmath==1.3.0',
        # Data
        'polars==1.38.1',
        # Networking / HTTP
        'requests==2.33.0',
        'urllib3==2.6.3',
        'certifi==2026.1.4',
        'charset-normalizer==3.4.4',
        'idna==3.11',
        # Utilities
        'tqdm==4.67.3',
        'psutil==7.2.2',
        'filelock==3.24.3',
        'fsspec==2026.2.0',
        'packaging==26.0',
        'PyYAML==6.0.3',
        'regex==2026.2.28',
        'ftfy==6.3.1',
        'wcwidth==0.6.0',
        'typing_extensions==4.15.0',
        # Templating
        'Jinja2==3.1.6',
        'MarkupSafe==3.0.3',
        # Graph / network
        'networkx==3.4.2',
        # Plotting deps
        'contourpy==1.3.2',
        'cycler==0.12.1',
        'fonttools==4.61.1',
        'kiwisolver==1.4.9',
        'pyparsing==3.3.2',
        'python-dateutil==2.9.0.post0',
        'six==1.17.0',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Object Detection Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.object_detection:main',
        ],
    },
)
