from setuptools import find_packages, setup

package_name = 'crazyflie_pointcloud'

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
    maintainer='ivans',
    maintainer_email='22964938@student.uwa.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiranger_to_pointcloud = crazyflie_pointcloud.multiranger_to_pointcloud:main',
        ],
    },
)
