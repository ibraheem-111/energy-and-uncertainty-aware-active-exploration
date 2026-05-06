import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

WORLD_DEFAULT = 'cave_simple_03'

IMU_GZ_TOPIC = '/world/cave_simple_03/model/x500_depth_modify_0/link/base_link/sensor/imu_sensor/imu'
RGB_CAMERA_GZ_TOPIC = '/rgb_camera'
DEPTH_CAMERA_GZ_TOPIC = '/depth_camera'
CAMERA_INFO_GZ_TOPIC = '/camera_info'
CAMERA_POINTS_GZ_TOPIC = '/depth_camera/points'

IMU_ROS_TOPIC = '/drone/imu'
RGB_CAMERA_ROS_TOPIC = '/drone/front_rgb'
DEPTH_CAMERA_ROS_TOPIC = '/drone/front_depth'
CAMERA_INFO_ROS_TOPIC = '/drone/camera_info'
CAMERA_POINTS_ROS_TOPIC = '/drone/front_depth/points'


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    world = LaunchConfiguration('world')

    use_sim_time_global = SetParameter(name='use_sim_time', value=True)

    gz_env = {
        'PX4_GZ_MODELS': [
            px4_dir,
            '/Tools/simulation/gz/models',
        ],
        'PX4_GZ_WORLDS': [
            px4_dir,
            '/Tools/simulation/gz/worlds',
        ],
        'PX4_GZ_PLUGINS': [
            px4_dir,
            '/build/px4_sitl_default/src/modules/simulation/gz_plugins',
        ],
        'PX4_GZ_SERVER_CONFIG': [
            px4_dir,
            '/src/modules/simulation/gz_bridge/server.config',
        ],
        'GZ_SIM_RESOURCE_PATH': [
            px4_dir,
            '/Tools/simulation/gz/models:',
            px4_dir,
            '/Tools/simulation/gz/worlds',
        ],
        'GZ_SIM_SYSTEM_PLUGIN_PATH': [
            px4_dir,
            '/build/px4_sitl_default/src/modules/simulation/gz_plugins',
        ],
        'GZ_SIM_SERVER_CONFIG_PATH': [
            px4_dir,
            '/src/modules/simulation/gz_bridge/server.config',
        ],
    }

    cleanup_sim = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                "pkill -f '[g]z sim' || true; "
                "pkill -f '[p]arameter_bridge .*cave_simple_03' || true; "
                "sleep 1"
            ),
        ],
        additional_env=gz_env,
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'{RGB_CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{CAMERA_INFO_GZ_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'{DEPTH_CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{CAMERA_POINTS_GZ_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'{IMU_GZ_TOPIC}@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[
            (RGB_CAMERA_GZ_TOPIC, RGB_CAMERA_ROS_TOPIC),
            (CAMERA_INFO_GZ_TOPIC, CAMERA_INFO_ROS_TOPIC),
            (DEPTH_CAMERA_GZ_TOPIC, DEPTH_CAMERA_ROS_TOPIC),
            (CAMERA_POINTS_GZ_TOPIC, CAMERA_POINTS_ROS_TOPIC),
            (IMU_GZ_TOPIC, IMU_ROS_TOPIC),
        ],
        output='screen',
    )

    ros_to_px4_odom = Node(
        package='cave_exploration',
        executable='ros_odom_to_px4_odom',
        name='ros_odom_to_px4_odom',
        output='screen',
    )

    start_gazebo_server = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '--verbose=1',
            '-r',
            '-s',
            [
                px4_dir,
                '/Tools/simulation/gz/worlds/',
                world,
                '.sdf',
            ],
        ],
        additional_env=gz_env,
        output='screen',
    )

    start_gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    start_gazebo_server_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_sim,
            on_exit=[start_gazebo_server],
        )
    )

    start_bridge_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_sim,
            on_exit=[bridge, start_gazebo_gui],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.path.join(
                os.environ.get(
                    'HOME',
                    f"/home/{os.environ.get('USER', 'user')}",
                ),
                'PX4-Autopilot',
            ),
            description='Path to the PX4-Autopilot repository.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=WORLD_DEFAULT,
            description='PX4 Gazebo world name (without .sdf).',
        ),
        use_sim_time_global,
        cleanup_sim,
        start_gazebo_server_after_cleanup,
        start_bridge_after_gazebo,
        ros_to_px4_odom,
    ])
