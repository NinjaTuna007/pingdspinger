import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pingdsp_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='ROS 2 driver for PingDSP 3DSS-DX sonar (TCP streaming)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tdss_driver = pingdsp_driver.tdss_driver:main',
            'pointcloud_filter = pingdsp_driver.pointcloud_filter:main',
            'pointcloud_recorder = pingdsp_driver.pointcloud_recorder:main',
        ],
    },
)
