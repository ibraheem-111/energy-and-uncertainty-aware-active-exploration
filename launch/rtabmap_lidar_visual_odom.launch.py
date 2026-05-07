from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_LINK_NAME = 'OakD-Lite-Modify/base_link'
BODY_LINK_NAME = 'base_link'
LIDAR_LINK_NAME = 'lidar_2d_link'
VISUAL_ODOM_TOPIC = '/rtabmap/visual_odom'
FUSED_ODOM_TOPIC = '/rtabmap/odom'
LIDAR_GZ_TOPIC = '/lidar_scan'
LIDAR_ROS_TOPIC = '/drone/lidar_scan'
CAMERA_POINTS_GZ_TOPIC = '/depth_camera/points'
CAMERA_POINTS_ROS_TOPIC = '/drone/front_depth/points'
IMU_ROS_TOPIC = '/drone/imu'
RGB_CAMERA_ROS_TOPIC = '/drone/front_rgb'
DEPTH_CAMERA_ROS_TOPIC = '/drone/front_depth'
CAMERA_INFO_ROS_TOPIC = '/drone/camera_info'

visual_odom_parameters = [{
    'use_sim_time': LaunchConfiguration('use_sim_time'),
    'frame_id': BODY_LINK_NAME,
    'approx_sync': True,
    'approx_sync_max_interval': 0.05,
    'topic_queue_size': 10,
    'queue_size': 10,
    'sync_queue_size': 10,
    'odom_sensor_sync': False,
    'odom_frame_id': 'visual_odom',
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
    'subscribe_scan': False,
}]

icp_odom_parameters = [{
    'use_sim_time': LaunchConfiguration('use_sim_time'),
    'frame_id': BODY_LINK_NAME,
    'odom_frame_id': 'odom',
    'guess_frame_id': 'visual_odom',
    'publish_tf': True,
    'wait_for_transform': 0.2,
    'subscribe_scan': True,
    'Reg/Force3DoF': 'true',
    'Icp/PointToPlane': 'false',
    'Icp/PM': 'false',
    'Icp/MaxCorrespondenceDistance': '0.15',
    'Icp/CorrespondenceRatio': '0.2',
    'Icp/MaxTranslation': '1.0',
    'Icp/VoxelSize': '0.05',
}]

slam_parameters = [{
    'use_sim_time': LaunchConfiguration('use_sim_time'),
    'frame_id': BODY_LINK_NAME,
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
    'subscribe_scan': True,
    'Reg/Strategy': '2',
    'Reg/Force3DoF': 'true',
    'RGBD/NeighborLinkRefining': 'True',
    'Grid/RayTracing': 'true',
    'Grid/3D': 'false',
    'Grid/Sensor': '2',
    'Grid/RangeMin': '0.2',
    'Optimizer/GravitySigma': '0',
}]

visual_odom_remappings = [
    ('rgb/image', RGB_CAMERA_ROS_TOPIC),
    ('rgb/camera_info', CAMERA_INFO_ROS_TOPIC),
    ('depth/image', DEPTH_CAMERA_ROS_TOPIC),
    ('odom', VISUAL_ODOM_TOPIC),
    ('imu', IMU_ROS_TOPIC),
]

icp_odom_remappings = [
    ('scan', LIDAR_ROS_TOPIC),
    ('odom', FUSED_ODOM_TOPIC),
]

slam_remappings = [
    ('rgb/image', RGB_CAMERA_ROS_TOPIC),
    ('rgb/camera_info', CAMERA_INFO_ROS_TOPIC),
    ('depth/image', DEPTH_CAMERA_ROS_TOPIC),
    ('odom', FUSED_ODOM_TOPIC),
    ('scan', LIDAR_ROS_TOPIC),
    ('imu', IMU_ROS_TOPIC),
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_visual_sensor_bridge',
            output='screen',
            arguments=[
                f'{LIDAR_GZ_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'{CAMERA_POINTS_GZ_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                (LIDAR_GZ_TOPIC, LIDAR_ROS_TOPIC),
                (CAMERA_POINTS_GZ_TOPIC, CAMERA_POINTS_ROS_TOPIC),
            ],
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_static_tf',
            arguments=[
                '-0.1',
                '0.0',
                '0.26',
                '0',
                '0',
                '0',
                'base_link',
                LIDAR_LINK_NAME,
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_visual_odometry',
            output='screen',
            parameters=visual_odom_parameters,
            remappings=visual_odom_remappings,
        ),
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='rtabmap_lidar_visual_odometry',
            output='screen',
            parameters=icp_odom_parameters,
            remappings=icp_odom_remappings,
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_lidar_visual_fusion',
            output='screen',
            arguments=['-d'],
            parameters=slam_parameters,
            remappings=slam_remappings,
        ),
        Node(
            package='cave_exploration',
            executable='ros_odom_to_px4_odom',
            name='lidar_visual_odom_to_px4',
            output='screen',
            parameters=[{
                'odom_topic': FUSED_ODOM_TOPIC,
                'publish_rate_hz': 30.0,
            }],
        ),
        Node(
            package = 'rtabmap_viz',
            executable = 'rtabmap_viz',
            name = 'rtabmap_viz',
            output = 'screen',
            parameters = visual_odom_parameters,
            remappings = visual_odom_remappings,
        ),
        LogInfo(
            msg='RTAB-Map lidar-assisted visual odometry launched'
        ),
    ])
