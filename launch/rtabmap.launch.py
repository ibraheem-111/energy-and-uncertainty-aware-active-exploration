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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
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
        LogInfo(
            msg='RTAB-Map odometry and slam launched with cave.launch.py-compatible settings'
        )
    ])
