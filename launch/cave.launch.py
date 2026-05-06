import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    UnlessCondition,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

X500_MODEL_DEFAULT = 'gz_x500_depth_modify'
WORLD_DEFAULT = 'cave_simple_03'

CAMERA_LINK_NAME = 'OakD-Lite-Modify/base_link'

RTABMAP_ODOM_TOPIC = '/rtabmap/odom'
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
    initial_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    model_pose_default = ','.join(str(x) for x in initial_pose)
    px4_dir = LaunchConfiguration('px4_dir')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    model_pose = LaunchConfiguration('model_pose', default=model_pose_default)
    sys_autostart = LaunchConfiguration('sys_autostart')
    gazebo_startup_delay = LaunchConfiguration('gazebo_startup_delay')

    # Apply sim time to every ROS node launched after this action.
    use_sim_time_global = SetParameter(name='use_sim_time', value=True)

    # The integrated cave launch owns the supported RTAB-Map path for this
    # vehicle, so keep a single shared parameter block aligned with the live
    # camera frame contract.
    parameters = [{
        'use_sim_time': True,
        'frame_id': CAMERA_LINK_NAME,
        'approx_sync': False,
        'queue_size': 10,
        'sync_queue_size': 10,
        'odom_sensor_sync': False,
        'odom_frame_id': 'odom',
        'visual_odometry': True,
        'publish_tf': True,
        'wait_for_transform': 0.2,
        'wait_imu_to_init': False,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_rgbd': False,
        'subscribe_odom': True,
    }]

    remappings = [
        ('rgb/image', RGB_CAMERA_ROS_TOPIC),
        ('rgb/camera_info', CAMERA_INFO_ROS_TOPIC),
        ('depth/image', DEPTH_CAMERA_ROS_TOPIC),
        ('odom', RTABMAP_ODOM_TOPIC),
        ('imu', IMU_ROS_TOPIC),
    ]

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
                "pkill -f '[m]ake px4_sitl' || true; "
                "pkill -f '[c]make --build .*/PX4-Autopilot/build/"
                "px4_sitl_default' || true; "
                "pkill -f '[n]inja gz_x500_depth' || true; "
                "pkill -f '[p]x4_sitl_default/bin/px4' || true; "
                "pkill -f '[g]z sim' || true; "
                "pkill -f '[p]arameter_bridge .*cave_simple_03' || true; "
                "pkill -f '[g]z_to_px4_odom' || true; "
                "pkill -f '[r]os_odom_to_px4_odom' || true; "
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
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Front RGB camera
            # '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
            # 'sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
            # 'sensor/IMX214/camera_info'
            # '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'{RGB_CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',

            # Camera info
            f'{CAMERA_INFO_GZ_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # Front depth camera
            f'{DEPTH_CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{CAMERA_POINTS_GZ_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # IMU
            f'{IMU_GZ_TOPIC}@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[
            (
            #     '/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu',
            #     '/drone/imu',
            # ),
                RGB_CAMERA_GZ_TOPIC,
                RGB_CAMERA_ROS_TOPIC,
            ),
            (
                CAMERA_INFO_GZ_TOPIC,
                CAMERA_INFO_ROS_TOPIC,
            ),
            (DEPTH_CAMERA_GZ_TOPIC, DEPTH_CAMERA_ROS_TOPIC),
            (CAMERA_POINTS_GZ_TOPIC, CAMERA_POINTS_ROS_TOPIC),
            (
                IMU_GZ_TOPIC,
                IMU_ROS_TOPIC,
            ),
        ],
        output='screen',
    )

    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_static_tf',
        arguments=[
            '0.12',
            '0.03',
            '0.242',
            '0',
            '0',
            '0',
            'base_link',
            CAMERA_LINK_NAME,
        ],
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
        cmd=[
            'gz',
            'sim',
            '-g',
        ],
        output='screen',
    )

    def px4_process(standalone):
        additional_env = {
            'PX4_GZ_WORLD': world,
            'PX4_GZ_MODEL_POSE': model_pose,
            'PX4_SYS_AUTOSTART': sys_autostart,
        }
        if standalone:
            additional_env['PX4_GZ_STANDALONE'] = '1'
            additional_env.update(gz_env)

        return ExecuteProcess(
            cmd=[
                'make', 'px4_sitl', model
            ],
            cwd=px4_dir,
            additional_env=additional_env,
            output='screen',
        )

    start_px4_after_world = px4_process(
        standalone=True,
    )

    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_rgbd_odometry',
        parameters=parameters,
        remappings=remappings,
        output='screen',
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['-d'],
        parameters=parameters,
        remappings=remappings
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=remappings,
    )

    rtabmap_to_px4_odom = Node(
        package='cave_exploration',
        executable='ros_odom_to_px4_odom',
        name='rtabmap_odom_to_px4',
        parameters=[{
            'odom_topic': RTABMAP_ODOM_TOPIC,
            'publish_rate_hz': 30.0,
        }],
        output='screen',
    )

    start_bridge_after_clean_px4 = RegisterEventHandler(
        OnProcessStart(
            target_action=start_px4_after_world,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[
                        bridge,
                        camera_static_tf,
                        rtabmap_odom,
                        rtabmap_slam,
                        rtabmap_viz,
                        rtabmap_to_px4_odom,
                    ],
                )
            ],
        )
    )

    start_gazebo_server_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_sim,
            on_exit=[start_gazebo_server],
        )
    )

    start_px4_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gazebo_server,
            on_start=[
                TimerAction(
                    period=gazebo_startup_delay,
                    actions=[
                        start_gazebo_gui,
                        start_px4_after_world,
                    ],
                )
            ],
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
            'model',
            default_value=X500_MODEL_DEFAULT,
            description='PX4 Gazebo model target passed to make px4_sitl.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=WORLD_DEFAULT,
            description='PX4 Gazebo world name (without .sdf).',
        ),
        DeclareLaunchArgument(
            'model_pose',
            default_value=model_pose_default,
            description='PX4 spawn pose x,y,z,roll,pitch,yaw.',
        ),
        DeclareLaunchArgument(
            'sys_autostart',
            default_value='4022',
            description=(
                'PX4 SYS_AUTOSTART airframe ID. '
                'Defaults to the cave RTAB-Map/external-vision airframe used '
                'by this integrated launch.'
            ),
        ),
        DeclareLaunchArgument(
            'gazebo_startup_delay',
            default_value='20.0',
            description=(
                'Seconds to let the cave world load before PX4 spawns the '
                'drone.'
            ),
        ),

        # Global sim time for all ROS nodes
        use_sim_time_global,
        cleanup_sim,
        start_gazebo_server_after_cleanup,
        start_px4_after_gazebo,
        start_bridge_after_clean_px4,
    ])
