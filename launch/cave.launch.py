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


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    model_pose = LaunchConfiguration('model_pose')
    sys_autostart = LaunchConfiguration('sys_autostart')
    clean_start = LaunchConfiguration('clean_start')
    gazebo_startup_delay = LaunchConfiguration('gazebo_startup_delay')

    # Apply sim time to every ROS node launched after this action.
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
        condition=IfCondition(clean_start),
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
            '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
            'sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
            'sensor/IMX214/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # Front depth camera
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # IMU
            '/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[
            (
            #     '/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu',
            #     '/drone/imu',
            # ),
                '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
                'sensor/IMX214/image',
                '/drone/front_rgb',
            ),
            (
                '/world/cave_simple_03/model/x500_depth_0/link/camera_link/'
                'sensor/IMX214/camera_info',
                '/drone/camera_info',
            ),
            ('/depth_camera', '/drone/front_depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            (
                '/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu',
                '/drone/imu',
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
            'camera_link',
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
        condition=IfCondition(clean_start),
        output='screen',
    )

    start_gazebo_gui = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-g',
        ],
        condition=IfCondition(clean_start),
        output='screen',
    )

    def px4_process(condition, standalone):
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
            condition=condition,
            output='screen',
        )

    start_px4 = px4_process(UnlessCondition(clean_start), standalone=False)
    start_px4_after_world = px4_process(
        IfCondition(clean_start),
        standalone=True,
    )

    gz_to_px4_odom = Node(
        package='cave_exploration',
        executable='gz_to_px4_odom',
        name='gz_to_px4_odom',
        condition=LaunchConfigurationEquals('odom_source', 'gz'),
        output='screen',
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_rgbd_odometry',
        condition=LaunchConfigurationEquals('odom_source', 'rtabmap'),
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'queue_size': 5,
            'sync_queue_size': 5,
            'wait_imu_to_init': False,
        }],
        remappings=[
            ('rgb/image', '/drone/front_rgb'),
            ('rgb/camera_info', '/drone/camera_info'),
            ('depth/image', '/drone/front_depth'),
            ('imu', '/drone/imu'),
            ('odom', '/rtabmap/odom'),
        ],
        output='screen',
    )

    rtabmap_common_parameters = [{
        'use_sim_time': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_rgbd': False,
        #'subscribe_odom': True,
        'visual_odometry':True,
        'approx_sync': True,
        'queue_size': 5,
        'sync_queue_size': 5,
        'wait_for_transform': 0.2,
    }]

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        condition=LaunchConfigurationEquals('odom_source', 'rtabmap'),
        output='screen',
        arguments=['-d'],
        parameters=rtabmap_common_parameters,
        remappings=[
            ('rgb/image', '/drone/front_rgb'),
            ('rgb/camera_info', '/drone/camera_info'),
            ('depth/image', '/drone/front_depth'),
            ('odom', '/rtabmap/odom'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        condition=LaunchConfigurationEquals('odom_source', 'rtabmap'),
        output='screen',
        parameters=rtabmap_common_parameters,
        remappings=[
            ('rgb/image', '/drone/front_rgb'),
            ('rgb/camera_info', '/drone/camera_info'),
            ('depth/image', '/drone/front_depth'),
            ('odom', '/rtabmap/odom'),
        ],
    )

    rtabmap_to_px4_odom = Node(
        package='cave_exploration',
        executable='ros_odom_to_px4_odom',
        name='rtabmap_odom_to_px4',
        condition=LaunchConfigurationEquals('odom_source', 'rtabmap'),
        parameters=[{
            'odom_topic': '/rtabmap/odom',
            'publish_rate_hz': 30.0,
        }],
        output='screen',
    )

    start_bridge_after_px4 = RegisterEventHandler(
        OnProcessStart(
            target_action=start_px4,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[
                        bridge,
                        camera_static_tf,
                        gz_to_px4_odom,
                        rgbd_odometry,
                        rtabmap,
                        rtabmap_viz,
                        rtabmap_to_px4_odom,
                    ],
                )
            ],
        )
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
                        gz_to_px4_odom,
                        rgbd_odometry,
                        rtabmap,
                        rtabmap_viz,
                        rtabmap_to_px4_odom,
                    ],
                )
            ],
        )
    )

    start_px4_after_cleanup = RegisterEventHandler(
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
            default_value='gz_x500_depth',
            description='PX4 Gazebo model target passed to make px4_sitl.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='cave_simple_03',
            description='PX4 Gazebo world name (without .sdf).',
        ),
        DeclareLaunchArgument(
            'model_pose',
            default_value='0,0,2,0,0,1.57',
            description='PX4 spawn pose x,y,z,roll,pitch,yaw.',
        ),
        DeclareLaunchArgument(
            'sys_autostart',
            default_value='4001',
            description=(
                'PX4 SYS_AUTOSTART airframe ID. '
                'Set this to your custom airframe/startup script ID for '
                'cave flight.'
            ),
        ),
        DeclareLaunchArgument(
            'odom_source',
            default_value='gz',
            choices=['gz', 'rtabmap'],
            description=(
                'Odometry source for PX4 external vision. '
                'gz uses Gazebo truth; rtabmap uses RGB-D odometry from '
                'the simulated sensors.'
            ),
        ),
        DeclareLaunchArgument(
            'clean_start',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Kill stale PX4/Gazebo/bridge processes before startup. '
                'Use false only when intentionally attaching to an existing '
                'Gazebo session.'
            ),
        ),
        DeclareLaunchArgument(
            'gazebo_startup_delay',
            default_value='20.0',
            description=(
                'Seconds to let the cave world load before PX4 spawns the '
                'drone in clean_start mode.'
            ),
        ),

        # Global sim time for all ROS nodes
        use_sim_time_global,
        cleanup_sim,
        start_px4,
        start_px4_after_cleanup,
        start_px4_after_gazebo,
        start_bridge_after_px4,
        start_bridge_after_clean_px4,
    ])
