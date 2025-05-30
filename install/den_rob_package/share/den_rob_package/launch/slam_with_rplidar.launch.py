from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start your custom odometry publisher node
        Node(
            package='den_rob_package',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),

        # Start RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{'frame_id': 'laser'}]  # ensure this matches your TF tree
        ),

        # Start SLAM Toolbox (online async mode)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # or 'async_slam_toolbox_node' if you prefer
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
