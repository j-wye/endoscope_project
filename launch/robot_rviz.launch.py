import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RViz 설정 파일의 경로를 구합니다. 'robot.rviz' 파일명을 사용합니다.
    rviz_config_file = os.path.join(
        get_package_share_directory('endoscope'),
        'rviz',  # rviz 설정 파일이 위치한 디렉토리
        'robot.rviz')  # 파일 이름

    # RViz 노드를 정의합니다.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    # LaunchDescription에 RViz 노드를 추가합니다.
    return LaunchDescription([
        rviz_node
    ])