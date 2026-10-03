"""Full simulation bringup for jgb_rover.

  ros2 launch jgb_rover_bringup bringup_sim.launch.py world:=apartment camera_pitch:=0.26 \
      slam:=true rviz:=true inject_errors:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from jgb_rover_control.spec_params import render
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(context, name) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def launch_setup(context):
    loc_share = get_package_share_directory('jgb_rover_localization')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('jgb_rover_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in
                          ('world', 'headless', 'camera_pitch', 'inject_errors')}.items())

    imu_bias = Node(
        package='jgb_rover_localization', executable='imu_bias_calibration', output='screen',
        parameters=[render(os.path.join(loc_share, 'config', 'imu_bias_calibration.yaml')),
                    {'use_sim_time': True}])

    ekf_overrides = {'use_sim_time': True}
    if as_bool(context, 'ekf_imu_accel'):
        ekf_overrides['imu0_config'] = [False, False, False, False, False, False,
                                        False, False, False, False, False, True,
                                        True, False, False]
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(loc_share, 'config', 'ekf.yaml'), ekf_overrides])

    return [sim, imu_bias, ekf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='apartment',
                              description='apartment | apartment_slip | path to .sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='rad down; empty = robot_spec.yaml (0.26)'),
        DeclareLaunchArgument('inject_errors', default_value='false',
                              description='left wheel 1.5 % smaller (sim_assumptions.yaml)'),
        DeclareLaunchArgument('ekf_imu_accel', default_value='false',
                              description='also fuse IMU forward acceleration in the EKF'),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        OpaqueFunction(function=launch_setup),
    ])
