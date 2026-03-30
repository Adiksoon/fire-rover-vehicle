import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, String, Bool


class PanTiltControllerNode(Node):
    def __init__(self):
        super().__init__("pan_tilt_controller_node")

        # SUBSKRYBCJE
        self.found_flag_sub = self.create_subscription(
            Bool, "/found_target_flag", self.flag_callback, 10
        )
        self.state_target_sub = self.create_subscription(
            String, "/search_detector/target_state", self.state_callback, 10
        )

        # PUBLIKACJE
        self.pan_pub = self.create_publisher(Float64, "/pan_tilt/pan_cmd", 10)
        self.tilt_pub = self.create_publisher(Float64, "/pan_tilt/tilt_cmd", 10)

        # INICJALIZACJE
        self.found_flag = False
        self.pan_position = 0.0
        self.target_state = "SEARCHING"
        self.previous_target_state = "SEARCHING"
        self.locked_pan_position = 0.0
        # self.tilt_position = 0.0
        self.pan_step = 0.02

        # ZEGAR WYWOŁUJĄCY METODE
        self.state_update = self.create_timer(0.1, self.machine_states)

    # CALLBACKS

    def flag_callback(self, msg):
        self.found_flag = msg.data

    def state_callback(self, msg):
        self.previous_target_state = self.target_state
        self.target_state = msg.data

        if (
            self.target_state == "CONFIRMED"
            and self.previous_target_state != "CONFIRMED"
        ):
            self.locked_pan_position = self.pan_position

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
        self.pan_position += self.pan_step * 0.25

        if self.pan_position > math.pi:
            self.pan_position = -math.pi

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
