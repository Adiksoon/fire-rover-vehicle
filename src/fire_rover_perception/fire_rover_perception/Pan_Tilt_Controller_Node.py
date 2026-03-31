import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import JointState


class PanTiltControllerNode(Node):
    def __init__(self):
        super().__init__("pan_tilt_controller_node")

        # SUBSKRYBCJE
        self.found_weak_flag_sub = self.create_subscription(
            Bool, "/search_detector/weak_target_flag", self.weak_flag_callback, 10
        )
        self.found_strong_flag_sub = self.create_subscription(
            Bool, "/search_detector/strong_target_flag", self.strong_flag_callback, 10
        )
        self.state_target_sub = self.create_subscription(
            String, "/search_detector/target_state", self.state_callback, 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # PUBLIKACJE
        self.pan_pub = self.create_publisher(Float64, "/pan_tilt/pan_cmd", 10)
        self.tilt_pub = self.create_publisher(Float64, "/pan_tilt/tilt_cmd", 10)

        # INICJALIZACJE
        self.weak_target_detected = False
        self.strong_target_detected = False

        self.pan_position = 0.0
        self.target_state = "SEARCHING"
        self.previous_target_state = "SEARCHING"
        self.locked_pan_position = 0.0
        # self.tilt_position = 0.0
        self.pan_step = 0.02

        self.current_pan_joint_position = 0.0
        self.candidate_center_pan = 0.0
        self.candidate_half_range = math.radians(60.0)
        self.candidate_direction = 1
        self.pan_joint_name = "pt_base_link_to_pt_link1"

        # ZEGAR WYWOŁUJĄCY METODE
        self.state_update = self.create_timer(0.1, self.machine_states)

    # CALLBACKS

    def weak_flag_callback(self, msg):
        self.weak_target_detected = msg.data

    def strong_flag_callback(self, msg):
        self.strong_target_detected = msg.data

    def state_callback(self, msg):
        self.previous_target_state = self.target_state
        self.target_state = msg.data

        if (
            self.target_state == "CANDIDATE"
            and self.previous_target_state != "CANDIDATE"
        ):
            self.candidate_center_pan = self.current_pan_joint_position

            self.pan_position = self.current_pan_joint_position

            self.candidate_direction = 1 if self.current_pan_joint_position > 0 else -1

        if (
            self.target_state == "CONFIRMED"
            and self.previous_target_state != "CONFIRMED"
        ):
            self.locked_pan_position = self.current_pan_joint_position

    def joint_states_callback(self, msg):
        if self.pan_joint_name in msg.name:
            idx = msg.name.index(self.pan_joint_name)
            self.current_pan_joint_position = msg.position[idx]

    # STATES
    ## STATE TREE

    def machine_states(self):

        if self.target_state == "SEARCHING":
            self.searching_behavior()
        elif self.target_state == "CANDIDATE":
            self.candidate_behavior()
        elif self.target_state == "CONFIRMED":
            self.confirmed_behavior()
        elif self.target_state == "LOST":
            self.lost_behavior()

    def searching_behavior(self):
        self.get_logger().info("State: SEARCHING")
        self.pan_position += self.pan_step

        if self.pan_position > math.pi:
            self.pan_position = -math.pi

        self.send_data(self.pan_position)

    def confirmed_behavior(self):
        self.get_logger().info("State: CONFIRMED")
        self.send_data(self.locked_pan_position)

    def candidate_behavior(self):
        self.get_logger().info("State: CANDIDATE")
        self.pan_position = self.current_pan_joint_position

        min_pan = self.candidate_center_pan - self.candidate_half_range
        max_pan = self.candidate_center_pan + self.candidate_half_range

        self.pan_position += self.pan_step * 0.25 * self.candidate_direction

        if self.pan_position > max_pan:
            self.pan_position = max_pan
            self.candidate_direction = -1
        elif self.pan_position < min_pan:
            self.pan_position = min_pan
            self.candidate_direction = 1

        self.send_data(self.pan_position)

    def lost_behavior(self):
        self.get_logger().info("State: LOST")
        self.pan_position += self.pan_step * 0.15

        if self.pan_position > math.pi:
            self.pan_position = -math.pi

        self.send_data(self.pan_position)

    def send_data(self, position):
        msg = Float64()
        msg.data = position
        self.pan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
