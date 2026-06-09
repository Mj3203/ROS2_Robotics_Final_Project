from setuptools import find_packages, setup

package_name = 'display_output_pkg'

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
    maintainer='mj3203',
    maintainer_email='mj3203@gmail.com',
    description='Display output: 2x2 camera grid plus game status bar in a single window',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'display_output = display_output_pkg.display_output_node:main',
        ],
    },
)
