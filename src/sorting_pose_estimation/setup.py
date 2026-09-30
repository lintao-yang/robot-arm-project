from setuptools import find_packages, setup

package_name = 'sorting_pose_estimation'

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
    description='模块3 目标定位：像素+深度 -> 基坐标系三维位置',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimator_node = sorting_pose_estimation.pose_estimator_node:main',
        ],
    },
)