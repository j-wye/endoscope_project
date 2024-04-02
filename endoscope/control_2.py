import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.twist = Twist()
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        #self.cmd_vel_straight = self.create_publisher(Twist, 'cmd_vel_straight', 10)
        self.cmd_vel_straight = self.create_publisher(TwistStamped, 'diff_drive_controller/cmd_vel', 10)
        # self.roll_pub = self.create_publisher(Float64, '/joint_between_chassis_controller/command', 10)
        # self.pitch_pub = self.create_publisher(Float64, '/joint_between_chassis_controller/command', 10)
        self.yaw_pub = self.create_publisher(Float64, '/velocity_controller/commands', 10)
        
    def joy_cb(self, msg):
        # twist_straight = Twist()
        # twist_straight.linear.x = msg.axes[5]
        # twist_straight.linear.y = 0.0
        # twist_straight.linear.z = 0.0
        # twist_straight.angular.x = 0.0
        # twist_straight.angular.y = 0.0
        # twist_straight.angular.z = 0.0
        # self.cmd_vel_straight.publish(twist_straight)

        twist_stamp = TwistStamped()
        twist_stamp.twist.linear.x = msg.axes[5]
        twist_stamp.twist.linear.y = 0.0
        twist_stamp.twist.linear.z = 0.0
        twist_stamp.twist.angular.x = 0.0
        twist_stamp.twist.angular.y = 0.0
        twist_stamp.twist.angular.z = 0.0
        self.cmd_vel_straight.publish(twist_stamp)

        # twist_steering = Twist()
        # twist_steering.linear.x = 0.0
        # twist_steering.linear.y = 0.0
        # twist_steering.linear.z = 0.0
        # twist_steering.angular.x = msg.axes[0]
        # twist_steering.angular.y = msg.axes[1]
        # twist_steering.angular.z = msg.axes[2]
        # self.cmd_vel_steering.publish(twist_steering)

        orientation_roll = Float64()
        orientation_pitch = Float64()
        orientation_yaw = Float64()
        orientation_roll.data = msg.axes[0]
        orientation_pitch.data = msg.axes[1]
        orientation_yaw.data = msg.axes[2]
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