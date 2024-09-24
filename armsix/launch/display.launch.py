import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
# https://github.com/argallab/jaco_ros2

def generate_launch_description():
    pkg_description = get_package_share_directory('armsix')

    # Process xacro file
    xacro_file = os.path.join(pkg_description, "urdf", 'armsix.urdf')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='',
            output='screen',
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_footprint_base',
        #     namespace='',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            namespace='',
            output='screen',
            arguments=['-d', os.path.join(pkg_description, 'rviz', 'urdf.rviz')],
        ),
    ])
