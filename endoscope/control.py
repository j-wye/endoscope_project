import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.twist = Twist()
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        
    def joy_cb(self, msg):
        twist_straight = Twist()
        twist_straight.linear.x = msg.axes[5]
        self.cmd_vel.publish(twist_straight)

        # JointTrajectory
        joint_traj = JointTrajectory()
        joint_traj.joint_names = ['front_chassis_roll_joint', 'front_chassis_pitch_joint', 'front_chassis_yaw_joint']
        point = JointTrajectoryPoint()
        
        # Assuming axes[0], axes[1], and axes[2] are used for roll, pitch, and yaw respectively
        point.positions = [float(msg.axes[0]), float(msg.axes[1]), float(msg.axes[2])]
        point.time_from_start.sec = 1  # Set the desired time from start (1 second for example)

        joint_traj.points.append(point)
        self.joint_traj_pub.publish(joint_traj)

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    rclpy.spin(joystick_control_node)
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()