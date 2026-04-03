import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math


class BaseAlignmentNode(Node):
    def __init__(self):
        super().__init__("base_alignment_node")
        self.get_logger().info("Base Alignment Node initialized")

        # SUBSKRYBCJE
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self.state_sub = self.create_subscription(
            String, "/search_detector/target_state", self.state_callback, 10
        )

        # PUBLIKACJE
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_align", 10)

        # INICJALIZACJA ZMIENNYCH
        self.pan_joint_angle = 0.0
        self.target_state = "SEARCHING"
        self.pan_joint_name = "pt_base_link_to_pt_link1"
        self.alignment_active = False

        # TIMER
        self.timer = self.create_timer(0.1, self.control_loop)

    def joint_states_callback(self, msg):
        if self.pan_joint_name in msg.name:
            idx = msg.name.index(self.pan_joint_name)
            self.pan_joint_angle = -msg.position[idx]

    def state_callback(self, msg):
        self.target_state = msg.data
        if self.target_state in ["CANDIDATE", "FOCUSED", "CONFIRMED"]:
            self.alignment_active = True
        else:
            self.alignment_active = False

    def control_loop(self):
        self.get_logger().info("START CONTROL LOOP")
        if not self.alignment_active:
            self.get_logger().info("Alignment inactive")
            return
        angle_error = self.pan_joint_angle
        if abs(angle_error) < 0.2:
            self.get_logger().info("Alignment complete! Stopping the robot.")
            return

        Kp = 0.6
        angular_z = Kp * angle_error
        max_speed = 1.0
        angular_z = max(-max_speed, min(max_speed, angular_z))
        self.get_logger().info(
            f"Angle error: {math.degrees(angle_error):.2f} degrees, Commanded angular.z: {angular_z:.2f}"
        )
        cmd = Twist()
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BaseAlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
