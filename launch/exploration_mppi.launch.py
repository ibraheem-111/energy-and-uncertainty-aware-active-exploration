from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("auto_engage", default_value="true"),
        DeclareLaunchArgument("marker_frame_id", default_value="map"),
        DeclareLaunchArgument("setpoint_speed_mps", default_value="0.12"),
        DeclareLaunchArgument("mppi_robot_radius_m", default_value="0.55"),
        DeclareLaunchArgument("mppi_max_heading_offset_deg", default_value="130.0"),
        DeclareLaunchArgument("mppi_enabled", default_value="true"),
        DeclareLaunchArgument("entry_distance_m", default_value="10.0"),
        DeclareLaunchArgument("experiment_label", default_value=""),
        DeclareLaunchArgument("experiment_timeout_s", default_value="300.0"),
        DeclareLaunchArgument("odom_info_topic", default_value="/rtabmap/odom_info"),
        DeclareLaunchArgument(
            "metrics_log_dir",
            default_value="/home/ibraheem/ros2_ws/exploration_logs",
        ),
        Node(
            package="cave_exploration",
            executable="exploration_offboard",
            name="exploration_offboard",
            output="screen",
            parameters=[{
                "auto_engage": ParameterValue(
                    LaunchConfiguration("auto_engage"),
                    value_type=bool,
                ),
                "marker_frame_id": LaunchConfiguration("marker_frame_id"),
                "entry_distance_m": ParameterValue(
                    LaunchConfiguration("entry_distance_m"),
                    value_type=float,
                ),
                "entry_lookahead_m": 5.0,
                "entry_setpoint_speed_mps": 0.8,
                "setpoint_speed_mps": ParameterValue(
                    LaunchConfiguration("setpoint_speed_mps"),
                    value_type=float,
                ),
                "vertical_setpoint_speed_mps": 0.45,
                "lidar_obstacle_avoidance_enabled": True,
                "lidar_avoid_distance_m": 0.25,
                "lidar_hard_stop_distance_m": 0.25,
                "lidar_side_clearance_m": 0.25,
                "lidar_emergency_stop_distance_m": 0.25,
                "mppi_enabled": ParameterValue(
                    LaunchConfiguration("mppi_enabled"),
                    value_type=bool,
                ),
                "mppi_robot_radius_m": ParameterValue(
                    LaunchConfiguration("mppi_robot_radius_m"),
                    value_type=float,
                ),
                "mppi_max_heading_offset_deg": ParameterValue(
                    LaunchConfiguration("mppi_max_heading_offset_deg"),
                    value_type=float,
                ),
                "frontier_goal_max_range_m": 8.0,
                "frontier_cluster_min_cells": 4,
                "frontier_distance_weight": -0.2,
                "frontier_forward_weight": 0.0,
                "mppi_frontier_candidate_limit": 250,
                "mppi_path_step_m": 0.35,
                "mppi_horizon_steps": 14,
                "mppi_heading_samples": 17,
                "mppi_allow_unknown_path_cells": True,
                "map_class_markers_enabled": True,
                "map_class_marker_stride_cells": 2,
                "frontier_marker_stride_cells": 1,
                "max_map_marker_cells": 6000,
                "metrics_enabled": True,
                "experiment_label": LaunchConfiguration("experiment_label"),
                "experiment_timeout_s": ParameterValue(
                    LaunchConfiguration("experiment_timeout_s"),
                    value_type=float,
                ),
                "odom_info_topic": LaunchConfiguration("odom_info_topic"),
                "metrics_log_dir": LaunchConfiguration("metrics_log_dir"),
                "metrics_period_s": 1.0,
            }],
        ),
    ])
