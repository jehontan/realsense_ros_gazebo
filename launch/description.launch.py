from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    param_robot_description = Command('xacro ', LaunchConfiguration('model'))
    
    return LaunchDescription([
      DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
          FindPackageShare('realsense_ros_gazebo'),
          'urdf',
          'text.xacro'
        ]),
        description='Absolute path to model URDF.'
      ),
      Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
          'robot_description': param_robot_description,
        }]
      ),
      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
          'robot_description': param_robot_description,
        }]
      ),
    ])