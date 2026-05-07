from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_LINK_NAME = 'OakD-Lite-Modify/base_link'
RTABMAP_ODOM_TOPIC = '/rtabmap/odom'
IMU_ROS_TOPIC = '/drone/imu'
RGB_CAMERA_ROS_TOPIC = '/drone/front_rgb'
DEPTH_CAMERA_ROS_TOPIC = '/drone/front_depth'
CAMERA_INFO_ROS_TOPIC = '/drone/camera_info'

parameters = [{
    'use_sim_time': LaunchConfiguration('use_sim_time'),
    'frame_id': 'base_link',
    'approx_sync': True,
    'approx_sync_max_interval': 0.05,
    'topic_queue_size': 10,
    'queue_size': 10,
    'sync_queue_size': 10,
    'odom_sensor_sync': False,
    'odom_frame_id': 'odom',
    'visual_odometry': True,
    'publish_tf': True,
    'wait_for_transform': 0.2,
    'wait_imu_to_init': True,
    'always_check_imu_tf': True,
    'qos_imu': 0,
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_static_tf',
            arguments=[
                '0.12',
                '0.03',
                '0.242',
                '-1.57079632679',
                '0',
                '-1.57079632679',
                'base_link',
                CAMERA_LINK_NAME,
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],
            parameters=parameters,
            remappings=remappings
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
        LogInfo(
            msg='RTAB-Map odometry and slam launched with cave.launch.py-compatible settings'
        )
    ])
