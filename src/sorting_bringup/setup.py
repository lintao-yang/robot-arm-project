from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sorting_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot sorting dev',
    maintainer_email='dev@example.com',
    description='系统级共享配置与顶层 launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf_relay = sorting_bringup.urdf_relay:main',
        ],
    },
)