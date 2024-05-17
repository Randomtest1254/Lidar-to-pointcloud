from setuptools import find_packages, setup

package_name = 'lidar_to_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='A package to convert LaserScan to PointCloud2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':[
            'lidar_to_pointcloud_node = lidar_to_pointcloud.lidar_to_pointcloud_node:main',
        ],
    },
)
