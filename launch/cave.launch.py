import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    model_pose = LaunchConfiguration('model_pose')

    cleanup_stale_sim = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            "pkill -9 -f '^gz sim' || true; pkill -9 -f '^.*/px4_sitl_default/bin/px4($| )' || true; rm -f /tmp/px4_lock-* || true",
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Front RGB Camera
            '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Front Depth Camera
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Clock and Odometry
            '/world/cave_simple_03/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/cave_simple_03/dynamic_pose/info@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',

            '/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            # Front RGB Camera remappings
            ('/rgb_camera', '/drone/front_rgb'),
            # ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            
            # Front Depth Camera remappings
            ('/depth_camera', '/drone/front_depth'),
            # ('/depth_camera/depth_image', '/drone/front_depth/depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/camera_info'),

            # Gazebo clock remapping
            ('/world/cave_simple_03/clock', '/clock'),
            
            # Odometry remapping
            ('/world/cave_simple_03/dynamic_pose/info', '/gz/world/cave_simple_03/dynamic_pose/info'),

            # IMU
            ('/world/cave_simple_03/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu', '/drone/imu'),
        ],
        output='screen'
    )

    start_px4 = ExecuteProcess(
        cmd=['make', 'px4_sitl', model],
        cwd=px4_dir,
        additional_env={
            'PX4_GZ_WORLD': world,
            'PX4_GZ_MODEL_POSE': model_pose,
        },
        output='screen',
    )

    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_stale_sim,
            on_exit=[start_px4],
        )
    )

    start_bridge_after_px4 = RegisterEventHandler(
        OnProcessStart(
            target_action=start_px4,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[bridge],
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
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
        cleanup_stale_sim,
        start_after_cleanup,
        start_bridge_after_px4,
    ])
