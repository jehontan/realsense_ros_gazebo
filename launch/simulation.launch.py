from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import  ParameterValue

def generate_launch_description():

    # Need to specify parameter type to overcome special chars in XML.
    # See https://github.com/ros2/launch_ros/issues/214
    param_robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    
    return LaunchDescription([
      DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
          FindPackageShare('realsense_ros_gazebo'),
          'urdf',
          'test.xacro'
        ]),
        description='Absolute path to model URDF.'
      ),

      ##### State publishers #####

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

      ##### Spawner #####

      Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=[
          '-z',
          '0.5',
          '-unpause',
          '-entity',
          'test_model',
          '-topic',
          '/robot_description'
        ],
      ),

      ##### Simulator #####

      # gzserver
      IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch','gzserver.launch.py']),
        launch_arguments=[
          ('pause', 'true'),
          ('record', 'false')
        ]
      ),
      # gzclient
      IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch','gzclient.launch.py'])
      ),

    ])