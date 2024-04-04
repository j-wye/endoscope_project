# endoscope_project

### Aliases
```bash
# aliases when using gazebo for endoscope
alias robot_rviz='sb && ros2 launch endoscope robot_rviz.launch.py'
alias sim='sb && cd ~/ros2_ws/ && cb && cd src/endoscope && cda && ros2 launch endoscope sim.launch.py world:=my_mesh.world'
alias sim_robot='sb && cd ~/ros2_ws/ && cb && cd src/endoscope && cda && ros2 launch endoscope sim.launch.py'
alias sim_robot_ros2='sb && cd ~/ros2_ws/ && cb && cd src/endoscope && cda && ros2 launch endoscope sim_ros2.launch.py'
alias key='sb && ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias joy0='sb && ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"'
alias joy1='sb && ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js1"'
alias control='sb && cd ~/ros2_ws/src/endoscope/endoscope && cda && python3 control.py'
alias control2='sb && cd ~/ros2_ws/src/endoscope/endoscope && cda && python3 control_2.py'
```

### Logs
- origin xyz, rpy에서 xyz가 먼저 적용되고 rpy가 적용!



### Progess
![screenshot](./rqt_graph.png)
![screenshot](./tf_tree.png)

#### References
[:link:](https://with-rl.tistory.com/entry/URDF%EB%A5%BC-%EC%9D%B4%EC%9A%A9%ED%95%9C-%EA%B0%84%EB%8B%A8%ED%95%9C-%EB%A1%9C%EB%B4%87-%EB%A7%8C%EB%93%A4%EA%B8%B0-3)

[:link:](https://davidvalenciaredro.wixsite.com/my-site/services-7)

[:link:](https://github.com/dvalenciar/robotic_arm_environment)

[:link:](https://github.com/ros-controls/gazebo_ros2_control/)

[:link:](https://github.com/ros-controls/ros2_controllers)

[:link:](https://robotics.snowcron.com/robotics_ros2/dif_drive_xacro.htm)

[:link:](https://robotics.snowcron.com/robotics_ros2/dif_drive_links.htm)
