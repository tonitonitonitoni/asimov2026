from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('sam_bot_description')
    
    ekf_local = os.path.join(pkg, 'config', 'ekf_local.yaml')
    ekf_global = os.path.join(pkg, 'config', 'ekf_global.yaml')
    navsat = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    return LaunchDescription([

        # EKF #1 (local odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local],
            remappings=[
                ('/odometry/filtered', '/odometry/local'),
            ]
        ),

        # navsat_transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat],
            remappings=[
                ('/imu', '/demo/imu'),
                ('/gps/fix', '/gps/fix'),
                ('/odometry/filtered', '/odometry/local'),
                ('/gps/filtered', '/gps/odom'),
            ]
        ),

        # EKF #2 (global)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
            ]
        ),
    ])
