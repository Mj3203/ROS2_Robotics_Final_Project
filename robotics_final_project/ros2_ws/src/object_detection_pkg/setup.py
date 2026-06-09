from setuptools import find_packages, setup

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/trained_models', 
         ['object_detection_pkg/trained_models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj3203',
    maintainer_email='mj3203@gmail.com',
    description='Object detection: YOLO piece detection and board state scanning',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_and_detect = object_detection_pkg.scan_and_detect_node:main',
        ],
    },
)
