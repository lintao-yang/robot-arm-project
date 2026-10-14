from setuptools import find_packages, setup

package_name = 'sorting_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot sorting dev',
    maintainer_email='dev@example.com',
    description='模块2.5 点云预处理：平面剔除 + 欧式聚类',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_preprocessor = '
            'sorting_perception.pointcloud_preprocessor:main',
            'dataset_capture = '
            'sorting_perception.dataset_capture_node:main',
            'yolo_seg_node = '
            'sorting_perception.yolo_seg_node:main',
        ],
    },
)