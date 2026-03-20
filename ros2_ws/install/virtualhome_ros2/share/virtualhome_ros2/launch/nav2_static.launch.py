import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'virtualhome_ros2'
    
    # Paths
    map_yaml_file = os.path.join(
        get_package_share_directory(package_name),
        'maps', 'real_room.yaml'
    )
    
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config', 'nav2_params.yaml'
    )

    return LaunchDescription([
        # --- VirtualHome ---
        Node(
            package='virtualhome_ros2',
            executable='vh_bridge_node',
            name='vh_bridge_node',
            output='screen',
        ),

        # --- Nav2 ---
        # 1. Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),

        # 2. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 3. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # 4. Lifecycle Manager (ACTIVATE ALL NODES!)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'behavior_server']
            }]
        ),

        # 5. Static TF Publisher: map -> odom (Zero Offset)
        # Prevents needing AMCL calculation freezes
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 6. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        )




    ])
