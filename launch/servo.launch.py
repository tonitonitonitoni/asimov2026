import os
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='sam_bot_description'
    ).find('sam_bot_description')

    default_model_path = os.path.join(
        pkg_share, 'src/description/sam_bot_servo_camera.urdf'
    )

    world_path = os.path.join(pkg_share, 'world/my_soccer_world.sdf')

    # -------------------------------
    # ROBOT STATE PUBLISHER
    # -------------------------------
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': True
        }],
        output='screen'
    )

    # -------------------------------
    # JOINT STATE PUBLISHER (OPTIONAL)
    # Not needed for ros2_control but harmless
    # -------------------------------
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------------
    # START GAZEBO CLASSIC
    # -------------------------------
    gazebo_start = launch.actions.ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # -------------------------------
    # SPAWN ENTITY (DELAYED)
    # -------------------------------
    spawn_entity = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'sam_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    delayed_spawn = launch.actions.TimerAction(
        period=3.0,  # wait for gazebo to load
        actions=[spawn_entity]
    )

    # -------------------------------
    # ros2_control CONTROLLERS
    # -------------------------------
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    camera_pan_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_pan_position_controller"],
        output="screen"
    )

    delayed_controllers = launch.actions.TimerAction(
        period=5.0,  # after robot spawns
        actions=[
            joint_state_broadcaster_spawner,
            camera_pan_controller_spawner,
        ]
    )

    display_heading = launch_ros.actions.Node(
        package='sam_bot_description',
        executable='display_heading.py',
        name='display_heading',
        output='screen'
    )


    # -------------------------------
    # FINAL LAUNCH DESCRIPTION
    # -------------------------------
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot URDF'
        ),

        gazebo_start,                   # Start Gazebo
        robot_state_publisher_node,     # Publish TF & joints
        joint_state_publisher_node,     # (optional)

        delayed_spawn,                  # Spawn robot
        delayed_controllers,            # Start ros2_control
        display_heading                 # view the camera feed
    ])
