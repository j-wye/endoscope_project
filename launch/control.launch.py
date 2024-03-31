from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='endoscope',  # ROS2 패키지 이름
            executable='joystick_control_node',  # 실행 파일 이름
            name='joystick_control_node',
            output='screen'
        )
    ])