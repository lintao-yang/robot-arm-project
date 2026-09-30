from setuptools import find_packages, setup

package_name = 'sorting_evaluation'

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
    description='模块7 日志与评价：指标记录与实验报告',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_logger_node = sorting_evaluation.metrics_logger_node:main',
            'make_report = sorting_evaluation.report:main',
        ],
    },
)