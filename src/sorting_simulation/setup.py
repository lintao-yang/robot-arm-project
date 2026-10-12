from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sorting_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot sorting dev',
    maintainer_email='dev@example.com',
    description='模块1 仿真环境与场景管理',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_manager = sorting_simulation.scene_manager:main',
            'dump_urdf = sorting_simulation.urdf_utils:main',
            'retry_spawner = sorting_simulation.retry_spawner:main',
        ],
    },
)