"""Bringup for the REAL jgb_rover: Teensy 4.1 board (wheels + MPU6050) over USB through ros2_control,
IMU bias calibration, EKF, and optionally the camera (USB webcam or RealSense D435), the floor scan
and slam_toolbox.
The same nodes and topics as bringup_sim.launch.py, with wall-clock time. No temperature sensor yet.

  ros2 launch jgb_rover_bringup bringup_real.launch.py                      # everything
  ros2 launch jgb_rover_bringup bringup_real.launch.py camera:=false        # drive + odometry only
  ros2 launch jgb_rover_bringup bringup_real.launch.py camera_type:=d435    # RealSense depth scan
  ros2 launch jgb_rover_bringup bringup_real.launch.py serial_port:=/dev/ttyACM0
  ros2 launch jgb_rover_bringup bringup_real.launch.py slow:=true             # walking pace

Keep the robot still for the first 3 s (gyro bias calibration).
"""
import os

from ament_index_python.packages import get_package_share_directory
import yaml
from jgb_rover_control.spec_params import load_spec, render, wall_clock
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def as_bool(context, name) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def with_slow_limits(param_path: str) -> str:
    """diff_drive_controller velocity / acceleration limits from robot_spec.yaml drive.slow_limits."""
    lim = load_spec()['drive']['slow_limits']
    with open(param_path) as f:
        data = yaml.safe_load(f)
    p = data['diff_drive_controller']['ros__parameters']
    for axis, vel, acc in (('linear.x', 'linear', 'linear_accel'), ('angular.z', 'angular', 'angular_accel')):
        p[f'{axis}.max_velocity'], p[f'{axis}.min_velocity'] = lim[vel], -lim[vel]
        p[f'{axis}.max_acceleration'], p[f'{axis}.min_acceleration'] = lim[acc], -lim[acc]
    out = os.path.join(os.path.dirname(param_path), 'slow_' + os.path.basename(param_path))
    with open(out, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return out


def launch_setup(context):
    desc_share = get_package_share_directory('jgb_rover_description')
    ctrl_share = get_package_share_directory('jgb_rover_control')
    loc_share = get_package_share_directory('jgb_rover_localization')
    wall = {'use_sim_time': False}
    d435 = LaunchConfiguration('camera_type').perform(context) == 'd435'

    robot_description = ParameterValue(Command(
        ['xacro ', os.path.join(desc_share, 'urdf', 'jgb_rover.urdf.xacro'),
         ' sim_gazebo:=false',
         ' camera_model:=', 'd435' if d435 else 'webcam',
         ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
         ' serial_port:=', LaunchConfiguration('serial_port')]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, wall])

    # The parameter files are shared with the simulation and set use_sim_time: true per node, which
    # beats a {'use_sim_time': False} dict (that one is a '/**' wildcard): wall_clock() rewrites them.
    # controllers_real.yaml adds the IMU broadcaster (later files override earlier ones).
    controllers = wall_clock(render(os.path.join(ctrl_share, 'config', 'controllers.yaml')))
    if as_bool(context, 'slow'):
        controllers = with_slow_limits(controllers)
    control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[{'robot_description': robot_description}, controllers,
                    os.path.join(ctrl_share, 'config', 'controllers_real.yaml')],
        remappings=[('/diff_drive_controller/cmd_vel', '/cmd_vel'),
                    ('/diff_drive_controller/odom', '/wheel/odom'),
                    ('/imu_sensor_broadcaster/imu', '/imu/data_raw')])

    spawners = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[name, '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '30'])
        for name in ('joint_state_broadcaster', 'diff_drive_controller', 'imu_sensor_broadcaster')]

    imu_bias = Node(
        package='jgb_rover_localization', executable='imu_bias_calibration', output='screen',
        parameters=[render(os.path.join(loc_share, 'config', 'imu_bias_calibration.yaml')), wall])

    ekf_overrides = dict(wall)
    if as_bool(context, 'ekf_imu_accel'):
        ekf_overrides['imu0_config'] = [False, False, False, False, False, False,
                                        False, False, False, False, False, True,
                                        True, False, False]
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[wall_clock(os.path.join(loc_share, 'config', 'ekf.yaml')), ekf_overrides])

    actions = [rsp, control_node, *spawners, imu_bias, ekf]

    camera = as_bool(context, 'camera')
    if camera and d435:
        # realsense2_camera (apt: ros-humble-realsense2-camera) publishes camera_depth_frame ... below
        # camera_link, which the URDF places on the mount. Depth only: the floor scan needs nothing else.
        rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_name': 'camera',
                'enable_depth': 'true', 'enable_color': LaunchConfiguration('d435_color'),
                'enable_infra1': 'false', 'enable_infra2': 'false', 'enable_gyro': 'false',
                'enable_accel': 'false', 'pointcloud.enable': 'false', 'align_depth.enable': 'false',
                'depth_module.profile': LaunchConfiguration('d435_depth_profile'),
                'rgb_camera.profile': LaunchConfiguration('d435_color_profile'),
                'initial_reset': 'true',      # a D435 left in a bad state by a killed process
            }.items()))
    elif camera:
        # open the camera at the size it was calibrated at (the WEB007 falls back to 640x360 anyway)
        camera_info_url = LaunchConfiguration('camera_info_url').perform(context)
        image_size = [640, 480]
        if camera_info_url.startswith('file://') and os.path.exists(camera_info_url[len('file://'):]):
            with open(camera_info_url[len('file://'):]) as f:
                info = yaml.safe_load(f)
            image_size = [info['image_width'], info['image_height']]
        actions.append(Node(
            package='v4l2_camera', executable='v4l2_camera_node', name='camera', output='screen',
            parameters=[{'video_device': LaunchConfiguration('video_device').perform(context),
                         'image_size': image_size,
                         'camera_frame_id': 'camera_optical_frame',
                         'camera_info_url': camera_info_url}],
            remappings=[('image_raw', '/camera/image_raw'), ('camera_info', '/camera/camera_info')]))
    if camera and as_bool(context, 'perception'):
        per_share = get_package_share_directory('jgb_rover_perception')
        if d435:
            actions.append(Node(
                package='jgb_rover_perception', executable='depth_floor_scan', output='screen',
                parameters=[os.path.join(per_share, 'config', 'depth_floor_scan.yaml'), wall]))
        else:
            actions.append(Node(
                package='jgb_rover_perception', executable='visual_floor_scan', output='screen',
                parameters=[os.path.join(per_share, 'config', 'visual_floor_scan.yaml'),
                            os.path.join(per_share, 'config', 'visual_floor_scan_real.yaml'), wall]))
        if as_bool(context, 'aruco') and not d435:
            actions.append(Node(
                package='jgb_rover_perception', executable='aruco_detector', output='screen',
                parameters=[os.path.join(per_share, 'config', 'aruco_detector.yaml'),
                            {'use_sim_time': False, 'ground_truth_file': ''}]))
        if as_bool(context, 'slam'):
            slam_params = wall_clock(LaunchConfiguration('slam_params_file').perform(context) or os.path.join(
                get_package_share_directory('jgb_rover_bringup'), 'config', 'slam_toolbox.yaml'))
            actions.append(Node(
                package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'use_sim_time': False,
                                          'use_scan_matching': as_bool(context, 'slam_scan_matching')}]))
    if as_bool(context, 'rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('jgb_rover_bringup'), 'rviz', 'sim.rviz')],
            parameters=[wall]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='',
                              description='MCU board serial port; empty = real_hardware.yaml (/dev/jgb_rover_mcu, '
                                          'falls back to /dev/ttyACM0)'),
        DeclareLaunchArgument('camera', default_value='true', description='camera + floor scan'),
        DeclareLaunchArgument('camera_type', default_value='webcam', description='webcam (v4l2_camera) | d435'),
        DeclareLaunchArgument('d435_depth_profile', default_value='640x480x15', description='WxHxFPS'),
        DeclareLaunchArgument('d435_color', default_value='false', description='D435 colour stream (not used yet)'),
        DeclareLaunchArgument('d435_color_profile', default_value='640x480x15'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url',
                              default_value='file://' + os.path.join(
                                  get_package_share_directory('jgb_rover_description'), 'config', 'webcam.yaml'),
                              description='camera calibration (jgb_rover_description/config/webcam.yaml)'),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='rad down; empty = robot_spec.yaml (0.26). Measure it on the robot'),
        DeclareLaunchArgument('perception', default_value='true', description='visual floor scan (needs camera)'),
        DeclareLaunchArgument('slam', default_value='true', description='slam_toolbox on /visual_scan'),
        DeclareLaunchArgument('slam_params_file', default_value=''),
        DeclareLaunchArgument('slam_scan_matching', default_value='false'),
        DeclareLaunchArgument('aruco', default_value='false', description='ArUco landmark detector'),
        DeclareLaunchArgument('ekf_imu_accel', default_value='false'),
        DeclareLaunchArgument('slow', default_value='false',
                              description='robot_spec.yaml drive.slow_limits instead of suggested_limits'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz (off: the Pi has no display)'),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        OpaqueFunction(function=launch_setup),
    ])
