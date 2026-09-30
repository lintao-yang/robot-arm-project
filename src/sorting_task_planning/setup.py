from setuptools import find_packages, setup

package_name = 'sorting_task_planning'

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
    description='任务调度与闭环状态机',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_planner_node = sorting_task_planning.task_planner_node:main',
        ],
    },
)