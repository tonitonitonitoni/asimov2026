import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='sam_bot_description').find('sam_bot_description')

    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/satellite_config.rviz')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')

    ekf_local_path = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_path = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_path = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    gui = LaunchConfiguration('gui')

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='spawner',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': Command(['xacro ', model])}],
        arguments=['-d', rvizconfig]
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'sam_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,  # wait for gazebo to load
        actions=[spawn_entity]
    )
    # EKF #1: local filter (odom frame)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
            # sensors are already on /demo/imu and /demo/odom, so no remap needed there
        ]
    )

    delayed_controllers = TimerAction(
        period=5.0,  # after robot spawns
        actions=[joint_state_publisher_node]
    )
    # navsat_transform_node: GPS (LLA) → ENU odom
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_path, {'use_sim_time': use_sim_time}],
        remappings=[
            # Inputs
            ('/imu', '/demo/imu'),
            ('/gps/fix', '/gps/fix_corrected'),
            ('/odometry/filtered', '/odometry/local'),
            # Outputs
            ('/gps/filtered', '/gps/odom'),
        ]
    )

    # EKF #2: global filter (map frame)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/global'),
        ]
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    gps_fix_restamper = Node(
        package='sam_bot_description',
        executable='gps_frame_fixer.py',
        name='gps_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    display_heading = Node(
        package='sam_bot_description',
        executable='display_heading.py',
        name='display_heading',
        output='screen'
    )

    path_pub = Node(
        package='sam_bot_description',
        executable='path_publisher.py',
        name='path_publisher',
        output='screen'
    )

    yolo_node = Node(
        package='sam_bot_description',
        executable='yolo_depth_v1.py',
        name='yolo_depth_v1',
        output='screen'
    )

    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens a new window 
        remappings=[
            ('/cmd_vel', '/demo/cmd_vel'),
        ]
    )
#TODO: ensure that all the packages from this file are added to the README and the CMakelists
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),

        gazebo,
        robot_state_publisher_node,
        delayed_spawn,
        delayed_controllers,
        gps_fix_restamper,
        navsat_node,
        ekf_local_node,
        ekf_global_node,
        display_heading,
        rviz_node,
        path_pub,
        yolo_node,
        teleop_twist_keyboard,
        foxglove,
    ])
