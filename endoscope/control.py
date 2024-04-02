import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.twist = Twist()
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.cmd_vel_straight = self.create_publisher(Twist, 'cmd_vel_straight', 10)
        # self.roll_pub = self.create_publisher(Float64, '/joint_between_chassis_controller/command', 10)
        # self.pitch_pub = self.create_publisher(Float64, '/joint_between_chassis_controller/command', 10)
        self.yaw_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
    def joy_cb(self, msg):
        twist_straight = Twist()
        twist_straight.linear.x = msg.axes[5]
        twist_straight.linear.y = 0.0
        twist_straight.linear.z = 0.0
        twist_straight.angular.x = 0.0
        twist_straight.angular.y = 0.0
        twist_straight.angular.z = 0.0
        self.cmd_vel_straight.publish(twist_straight)

        orientation_roll = Float64MultiArray()
        orientation_pitch = Float64MultiArray()
        orientation_yaw = Float64MultiArray()
        orientation_roll.data = [float(msg.axes[0])]
        orientation_pitch.data = [float(msg.axes[1])]
        orientation_yaw.data = [float(msg.axes[2])]
        # self.roll_pub.publish(orientation_roll)
        # self.pitch_pub.publish(orientation_pitch)
        self.yaw_pub.publish(orientation_yaw)

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    rclpy.spin(joystick_control_node)
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()