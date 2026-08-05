from setuptools import find_packages, setup

package_name = 'computer_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/trained_models',
         ['computer_vision_pkg/trained_models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj3203',
    maintainer_email='mj3203@gmail.com',
    description='Computer vision: raw camera capture and ArUco homography board warping',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'raw_camera_feed = computer_vision_pkg.raw_camera_feed_node:main',
            'homography_transform = computer_vision_pkg.homography_transform_node:main',
            'scan_and_detect = computer_vision_pkg.scan_and_detect_node:main',
        ],
    },
)
