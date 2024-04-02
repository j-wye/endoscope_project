import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='endoscope'

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_exe.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'colonoscopy'],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node', 
        name='joystick',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    # load_diff_drive_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        robot,
        gazebo,
        spawn_entity,
        joint_state_publisher,
        joy_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])
