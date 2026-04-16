import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
	EnvironmentVariable,
	LaunchConfiguration,
	PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    model = LaunchConfiguration('model')

    spawn_cave = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments=[
            '-file', os.path.join(
                get_package_share_directory('cave_exploration'),
                'cave_simple_03',
                'simple_cave_03.sdf',
            ),
            '-name', 'cave',
        ]
    )

    delayed_spawn_cave = TimerAction(
        period=2.0,
        actions=[spawn_cave]
    )

    return LaunchDescription([        
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to the PX4-Autopilot repository.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='gz_x500_depth_mono',
            description='PX4 Gazebo model target passed to make px4_sitl.',
        ),
        ExecuteProcess(
            cmd=['make', 'px4_sitl', model],
            cwd=px4_dir,
            
            output='screen',
        ),
        # delayed_spawn_cave,
    ])
