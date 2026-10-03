"""Robot model only (no simulator): robot_state_publisher + joint_state_publisher (+ RViz).

  ros2 launch jgb_rover_description display.launch.py camera_pitch:=0.26 gui:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('jgb_rover_description')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'jgb_rover.urdf.xacro'),
        ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
        ' inject_errors:=', LaunchConfiguration('inject_errors'),
        ' sim_gazebo:=false']), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('camera_pitch', default_value='', description='empty = robot_spec.yaml'),
        DeclareLaunchArgument('inject_errors', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false', description='joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
