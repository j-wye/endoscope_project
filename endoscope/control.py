import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.twist = Twist()
        self.bridge = CvBridge()
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, self.qos_policy)
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.img_cb, self.qos_policy)
        self.depth_img_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_img_cb, self.qos_policy)

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', self.qos_policy)
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.qos_policy)

        self.should_continue = True

    def joy_cb(self, msg):
        twist_straight = Twist()
        twist_straight.linear.x = msg.axes[5]
        twist_straight.angular.z = msg.axes[4]
        self.cmd_vel.publish(twist_straight)

        joint_pose = JointTrajectory()
        point = JointTrajectoryPoint()
        joint_pose.joint_names = ['chassis_roll_joint', 'chassis_pitch_joint', 'chassis_yaw_joint']

        point.positions = [float(msg.axes[0]), float(msg.axes[1]), float(msg.axes[2])]
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
    #     # depth_img = np.where(np.isinf(depth_img), np.nan, depth_img)
    #     # max_depth = np.nanmax(depth_img) if np.nanmax(depth_img) > 0 else 1
        
    #     # max_depth = 100.0
    #     # # 깊이 값을 0-255 범위로 정규화합니다.
    #     # depth_img_normalized = np.nan_to_num(depth_img, nan=max_depth)
    #     # depth_img_normalized = ((depth_img_normalized / max_depth) * 255).astype(np.uint8)
        
    #     # cv2.imshow("Depth Image", depth_img_normalized)
    #     # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     #     self.should_continue = False

    #     max_depth = 100.0  # 깊이의 최대값을 100.0으로 설정
    #     # 깊이 값을 0-255 범위로 정규화합니다.
    #     depth_img_normalized = np.nan_to_num(depth_img, nan=max_depth)
    #     depth_img_normalized = ((depth_img_normalized / max_depth) * 255).astype(np.uint8)
        
    #     cv2.imshow("Depth Image", depth_img_normalized)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         self.should_continue = False

    def depth_img_cb(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth Camera CV Bridge Error : {e}")
            return
        
        max_depth = 100.0  # 깊이의 최대값을 100.0으로 설정
        for i in range(480):
            for j in range(640):
                if depth_img[i][j] == "inf":
                    depth_img[i][j] = max_depth
                    #print(depth_img[i][j])

        # 깊이 값이 0에 가까운 경우에 대한 처리를 추가합니다.
        # 여기서는 예시로 0.1 미터보다 가까운 모든 값들을 0.1로 설정합니다.
        # depth_img_clipped = np.clip(depth_img, 0.1, max_depth)
        
        # 깊이 값을 0-255 범위로 정규화합니다.
        depth_img_normalized = ((depth_img / max_depth) * 255)#.astype(np.uint8)
        
        cv2.imshow("Depth Image", depth_img_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.should_continue = False

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