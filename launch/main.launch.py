from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    main_node = Node(
        package='hit_mechanism2',
        executable='main',
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('hit_mechanism2'), 
            'config', 'main.rviz'])
            ]
    )
    return LaunchDescription([
        main_node,
        rviz2_node,
    ])