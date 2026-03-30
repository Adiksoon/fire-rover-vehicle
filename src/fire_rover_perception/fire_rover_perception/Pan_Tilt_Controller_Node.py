import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import math
from std_msgs.msg import Float64


class PanTiltControllerNode(Node):
    def __init__(self):
        super().__init__("pan_tilt_controller_node")

        # SUBSKRYBCJE
        self.found_flag_sub = self.create_subscription(
            Bool, "/flag", self.flag_callback, 10
        )
        # PUBLIKACJE
        self.pan_pub = self.create_publisher(Float64, "/pan_tilt/pan_cmd", 10)
        self.tilt_pub = self.create_publisher(Float64, "/pan_tilt/tilt_cmd", 10)

        # INICJALIZACJE
        self.found_flag = False
        self.pan_position = 0.0
        # self.tilt_position = 0.0
        self.pan_step = 0.05

        # ZEGAR WYWOŁUJĄCY METODE
        self.state_update = self.create_timer(0.1, self.state_update_callback)

    # CALLBACKS

    def flag_callback(self, msg):
        self.found_flag = msg.data

    # METHODS

    def state_update_callback(self):
        if self.found_flag:
            return
        self.pan_position += self.pan_step

        if self.pan_position > math.pi:
            self.pan_position = -math.pi

        msg = Float64()
        msg.data = self.pan_position
        self.pan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
