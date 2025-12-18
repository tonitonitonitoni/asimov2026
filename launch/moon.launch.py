import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sam_bot_description')
    model_path = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{model_path}:{existing_model_path}" if existing_model_path else model_path
    world_path = os.path.join(pkg_share, 'world', 'my_moon_world.sdf')

    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    ekf_local_path = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_path = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_path = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
        ]
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu', '/demo/imu'),
            ('/gps/fix', '/gps/fix_corrected'),
            ('/odometry/filtered', '/odometry/local'),
            ('/gps/filtered', '/gps/odom'),
        ]
    )

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

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path}
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
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gps_fix_restamper,
        navsat_node,
        ekf_local_node,
        ekf_global_node,
        display_heading,
    ])
