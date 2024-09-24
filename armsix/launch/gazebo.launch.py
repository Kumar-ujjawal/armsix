import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('robot_description', default_value='package://armsix/urdf/armsix.urdf'),
        DeclareLaunchArgument('world_name', default_value='package://gazebo_ros/worlds/empty.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        
        # Include the Gazebo launch file to start an empty world
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            arguments=[
                '--world', LaunchConfiguration('world_name'),
                '--pause', LaunchConfiguration('paused'),
                '--headless', LaunchConfiguration('headless'),
                '--verbose', LaunchConfiguration('debug')
            ],
            output='screen'
        ),
        
        # Load the robot description into the parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'armsix',
                '-file', LaunchConfiguration('robot_description'),
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
        
        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        # Controller Manager Node
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_manager',
            output='screen',
            arguments=[
                'joint_state_controller',
                'joint_1_position_controller',
                'joint_2_position_controller',
                'joint_3_position_controller',
                'joint_4_position_controller',
                'joint_5_position_controller',
                'joint_6_position_controller',
                'joint_trajectory_controller'
            ]
        ),
        
        # Load the controller configuration
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_manager',
            output='screen',
            arguments=['controller_manager'],
            parameters=[ParameterFile('path/to/control.yaml')]
        ),
    ])
