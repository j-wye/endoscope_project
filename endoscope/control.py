import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TransformStamped
import tf2_tools.view_frames
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf2_tools

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.twist = Twist()
        self.bridge = CvBridge()
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, self.qos_policy)
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.img_cb, self.qos_policy)
        # self.depth_img_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_img_cb, self.qos_policy)
        # self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_cb, self.qos_policy)

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', self.qos_policy)
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.qos_policy)
        self.tf_pub = self.create_publisher(TFMessage, '/asdasd', self.qos_policy)

        self.should_continue = True

    def joy_cb(self, msg):
        accel = 2.0
        twist_straight = Twist()
        if msg.buttons[0]:
            twist_straight.linear.x = msg.axes[5] * accel
            twist_straight.angular.z = msg.axes[4] * 0.3 * accel
        else:
            twist_straight.linear.x = msg.axes[5]
            twist_straight.angular.z = msg.axes[4] * 0.3
        self.cmd_vel.publish(twist_straight)

        joint_pose = JointTrajectory()
        point = JointTrajectoryPoint()
        joint_pose.joint_names = ['chassis_roll_joint', 'chassis_pitch_joint', 'chassis_yaw_joint']

        point.positions = [float(msg.axes[0] * 0.5), float(msg.axes[1] * 0.5), float(msg.axes[2] * 0.5)]
        point.time_from_start.sec = 1

        joint_pose.points.append(point)
        self.joint_pub.publish(joint_pose)
    
    def img_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error : {e}")
            return
        cv2.imshow("Colonoscopy Camera Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.should_continue = False

    # def depth_img_cb(self, msg):
    #     try:
    #         depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    #     except CvBridgeError as e:
    #         self.get_logger().error(f"Depth Camera CV Bridge Error : {e}")
    #         return
        
    #     max_depth = 100.0  # 깊이의 최대값을 100.0으로 설정
        
    #     depth_img_normalized = ((depth_img / max_depth) * 255)
        
    #     cv2.imshow("Depth Image", depth_img_normalized)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         self.should_continue = False

#     def print_tf(self, tfs):


#     def tf_cb(self, msg):
#         tf_message = TFMessage()
#         ts = TransformStamped()
#         #alpha = ts.transform.rotation.z
        
#         print(msg.transforms)
#         #self.tf_pub.publish(tf2)

# msg.transforms = 
# [
#     geometry_msgs.msg.TransformStamped(
#         header=std_msgs.msg.Header(
#             stamp=builtin_interfaces.msg.Time(sec=3571, nanosec=295000000), 
#             frame_id='chassis_roll_link'), 
#         child_frame_id='chassis_pitch_link', 
#         transform=geometry_msgs.msg.Transform(
#             translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), 
#             rotation=geometry_msgs.msg.Quaternion(x=0.0, y=1.5959455978986625e-16, z=0.0, w=1.0))), 
#     geometry_msgs.msg.TransformStamped(
#         header=std_msgs.msg.Header(
#             stamp=builtin_interfaces.msg.Time(sec=3571, nanosec=295000000), 
#             frame_id='back_chassis'), 
#         child_frame_id='chassis_roll_link', 
#         transform=geometry_msgs.msg.Transform(
#             translation=geometry_msgs.msg.Vector3(x=0.8125, y=0.0, z=0.0), 
#             rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), 
# ]


def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    while joystick_control_node.should_continue:
        rclpy.spin_once(joystick_control_node, timeout_sec=0.1)
    joystick_control_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# tf2_msgs.msg.TFMessage(
#     transforms=[
#         geometry_msgs.msg.TransformStamped(
#             header=std_msgs.msg.Header(
#                 stamp=builtin_interfaces.msg.Time(sec=2739, nanosec=904000000), 
#                 frame_id='odom'),
#             child_frame_id='base_link'
#             transform=geometry_msgs.msg.Transform(
#                 translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0),
#                 rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#             )
#         ),
#     ]
# )