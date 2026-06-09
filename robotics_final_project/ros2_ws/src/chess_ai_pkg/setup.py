from setuptools import find_packages, setup
import os 

package_name = 'chess_ai_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'stockfish_engine'), 
            ['stockfish_engine/stockfish-ubuntu-x86-64-avx2']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj3203',
    maintainer_email='mj3203@gmail.com',
    description='Chess AI: Stockfish engine + python-chess move validation and board state',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chess_ai = chess_ai_pkg.chess_ai_node:main'
        ],
    },
)
