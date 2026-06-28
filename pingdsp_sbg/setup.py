from glob import glob
import os

from setuptools import find_packages, setup


def _files(pattern):
    """Return only regular files matching pattern (skip __pycache__ etc.)."""
    return [f for f in glob(pattern) if os.path.isfile(f)]


package_name = 'pingdsp_sbg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', _files('launch/*')),
        ('share/' + package_name + '/config', _files('config/*')),
        ('share/' + package_name + '/scripts', _files('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='SBG Ellipse INS integration (odom + TF) for the PingDSP rig',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sbg_to_odom = pingdsp_sbg.sbg_to_odom:main',
            'sbg_to_odom_initializer = '
            'pingdsp_sbg.sbg_to_odom_initializer:main',
        ],
    },
)
