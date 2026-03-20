from setuptools import find_packages, setup

package_name = 'virtualhome_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_static.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/maps', ['maps/real_room.pgm', 'maps/real_room.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selina-xiangqi',
    maintainer_email='2107931860@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vh_bridge_node = virtualhome_ros2.vh_bridge_node:main',
            'teleop_node = virtualhome_ros2.teleop_node:main'
        ],
    },
)
