"""Full simulation bringup for jgb_rover: Gazebo, ros2_control, IMU bias calibration, EKF,
visual floor scan, slam_toolbox, simulated MCP9808 + temperature heatmap, optional ArUco check,
path recorder and RViz.

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


def world_is_apartment(context) -> bool:
    return LaunchConfiguration('world').perform(context) in ('apartment', 'apartment_slip')


def launch_setup(context):
    loc_share = get_package_share_directory('jgb_rover_localization')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('jgb_rover_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in
                          ('world', 'headless', 'headless_rendering', 'camera_pitch', 'inject_errors', 'x', 'y', 'yaw')}.items())

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

    actions = [sim, imu_bias, ekf]
    if as_bool(context, 'perception'):
        per_share = get_package_share_directory('jgb_rover_perception')
        actions.append(Node(
            package='jgb_rover_perception', executable='visual_floor_scan', output='screen',
            parameters=[os.path.join(per_share, 'config', 'visual_floor_scan.yaml'), {'use_sim_time': True}]))
    spawn = [float(LaunchConfiguration(k).perform(context)) for k in ('x', 'y', 'yaw')]
    actions.append(Node(
        package='jgb_rover_bringup', executable='path_recorder.py', output='screen',
        parameters=[{'use_sim_time': True, 'spawn_pose': spawn}]))
    if as_bool(context, 'rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('jgb_rover_bringup'), 'rviz', 'sim.rviz')],
            parameters=[{'use_sim_time': True}]))
    if as_bool(context, 'aruco'):
        per_share = get_package_share_directory('jgb_rover_perception')
        layout = os.path.join(get_package_share_directory('jgb_rover_gazebo'), 'config', 'apartment_layout.yaml')
        actions.append(Node(
            package='jgb_rover_perception', executable='aruco_detector', output='screen',
            parameters=[os.path.join(per_share, 'config', 'aruco_detector.yaml'),
                        {'use_sim_time': True, 'spawn_pose': spawn,
                         'ground_truth_file': layout if world_is_apartment(context) else ''}]))
    if as_bool(context, 'temperature'):
        tmp_share = get_package_share_directory('jgb_rover_temperature')
        layout = os.path.join(get_package_share_directory('jgb_rover_gazebo'), 'config', 'apartment_layout.yaml')
        actions.append(Node(
            package='jgb_rover_temperature', executable='sim_mcp9808', output='screen',
            parameters=[render(os.path.join(tmp_share, 'config', 'mcp9808_driver.yaml')),
                        {'use_sim_time': True, 'layout_file': layout if world_is_apartment(context) else ''}]))
        actions.append(Node(
            package='jgb_rover_temperature', executable='temperature_mapper', output='screen',
            parameters=[render(os.path.join(tmp_share, 'config', 'temperature_mapper.yaml')),
                        {'use_sim_time': True, 'map_topic': '/map' if as_bool(context, 'slam') else ''}]))
    if as_bool(context, 'slam'):
        slam_params = LaunchConfiguration('slam_params_file').perform(context) or os.path.join(
            get_package_share_directory('jgb_rover_bringup'), 'config', 'slam_toolbox.yaml')
        actions.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
            parameters=[slam_params, {'use_sim_time': True,
                                      'use_scan_matching': as_bool(context, 'slam_scan_matching')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='apartment',
                              description='apartment | apartment_slip | path to .sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='EGL rendering for machines without a display'),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='rad down; empty = robot_spec.yaml (0.26)'),
        DeclareLaunchArgument('inject_errors', default_value='false',
                              description='left wheel 1.5 % smaller (sim_assumptions.yaml)'),
        DeclareLaunchArgument('ekf_imu_accel', default_value='false',
                              description='also fuse IMU forward acceleration in the EKF'),
        DeclareLaunchArgument('perception', default_value='true', description='visual floor scan'),
        DeclareLaunchArgument('slam', default_value='true', description='slam_toolbox on /visual_scan'),
        DeclareLaunchArgument('slam_params_file', default_value='',
                              description='slam_toolbox params; empty = jgb_rover_bringup/config/slam_toolbox.yaml'),
        DeclareLaunchArgument('slam_scan_matching', default_value='false',
                              description='let slam_toolbox correct the EKF pose by scan matching (see README)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz with map, scan, paths, debug image'),
        DeclareLaunchArgument('temperature', default_value='true',
                              description='simulated MCP9808 + temperature heatmap (/temperature_map)'),
        DeclareLaunchArgument('aruco', default_value='false', description='ArUco landmark detector + map check'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        OpaqueFunction(function=launch_setup),
    ])
