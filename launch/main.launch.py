from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution,Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    main_node = Node(
        package='hit_mechanism2',
        executable='main',
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('hit_mechanism2'),
        'urdf',
        'cam.SLDASM.urdf'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
    )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher_node',
            parameters=[{'robot_description': robot_description}],
        )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('hit_mechanism2'), 
            'config', 'rviz.rviz'])
            ]
    )
    return LaunchDescription([
        main_node,
        robot_state_publisher_node,
        rviz2_node,
    ])