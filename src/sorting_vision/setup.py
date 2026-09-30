from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sorting_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot sorting dev',
    maintainer_email='dev@example.com',
    description='模块2 视觉感知：YOLO11-seg 检测与合成数据生成',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = sorting_vision.detector_node:main',
            'synthetic_dataset = sorting_vision.synthetic_dataset:main',
        ],
    },
)