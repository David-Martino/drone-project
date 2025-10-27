from setuptools import setup
import os
from glob import glob


package_name = 'cf3d_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='3D mapping/planning/controller for CF',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cf3d_voxelizer=cf3d_nav.voxelizer:main',
            'explorer3d=cf3d_nav.explorer3d:main',



        ],
    },
)
