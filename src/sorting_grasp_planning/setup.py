from setuptools import find_packages, setup

package_name = 'sorting_grasp_planning'

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
    description='模块4 抓取决策：四段式抓取位姿生成',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_planner_node = sorting_grasp_planning.grasp_planner_node:main',
        ],
    },
)