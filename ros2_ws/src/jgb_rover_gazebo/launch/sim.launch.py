"""Start Gazebo Fortress with a world, spawn jgb_rover and bridge the sim topics.

Example:
  ros2 launch jgb_rover_gazebo sim.launch.py world:=apartment headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    gz_share = get_package_share_directory('jgb_rover_gazebo')
    desc_share = get_package_share_directory('jgb_rover_description')

    world = LaunchConfiguration('world').perform(context)
    world_path = world if world.endswith('.sdf') else os.path.join(gz_share, 'worlds', f'{world}.sdf')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    gz_args = f'-r {"-s --headless-rendering " if headless else ""}{world_path}'

    xacro_file = os.path.join(desc_share, 'urdf', 'jgb_rover.urdf.xacro')
    robot_description = ParameterValue(Command(
        ['xacro ', xacro_file,
         ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
         ' inject_errors:=', LaunchConfiguration('inject_errors'),
         ' controllers_file:=', LaunchConfiguration('controllers_file')]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'jgb_rover', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])

    return [gazebo, rsp, spawn, bridge]


def generate_launch_description():
    gz_share = get_package_share_directory('jgb_rover_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='apartment',
                              description='World name in jgb_rover_gazebo/worlds or a path to an .sdf'),
        DeclareLaunchArgument('headless', default_value='false', description='Run the server only, no GUI'),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='Camera pitch down [rad]; empty = robot_spec.yaml default'),
        DeclareLaunchArgument('inject_errors', default_value='false',
                              description='Left wheel radius mismatch (see sim_assumptions.yaml)'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='ros2_control controllers yaml; empty = jgb_rover_control default'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.005', description='Small drop so the robot settles'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # The pip numpy 2 / OpenCV 5 in ~/.local break cv_bridge; always use the system packages.
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(gz_share, 'models')),
        OpaqueFunction(function=launch_setup),
    ])
