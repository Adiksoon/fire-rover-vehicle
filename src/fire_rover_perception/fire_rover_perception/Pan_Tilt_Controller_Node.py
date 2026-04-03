import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R


class PanTiltControllerNode(Node):
    def __init__(self):
        super().__init__("pan_tilt_controller_node")

        # SUBSKRYBCJE
        self.weak_flag_sub = self.create_subscription(
            Bool, "/search_detector/weak_target_flag", self.weak_flag_callback, 10
        )
        self.strong_flag_sub = self.create_subscription(
            Bool, "/search_detector/strong_target_flag", self.strong_flag_callback, 10
        )
        self.state_target_sub = self.create_subscription(
            String, "/search_detector/target_state", self.state_callback, 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self.error_sub = self.create_subscription(
            Point, "/error_xy", self.error_callback, 10
        )

        # PUBLIKACJE
        self.pan_pub = self.create_publisher(Float64, "/pan_tilt/pan_cmd", 10)
        self.tilt_pub = self.create_publisher(Float64, "/pan_tilt/tilt_cmd", 10)

        # INICJALIZACJE
        self.weak_flag = False
        self.strong_flag = False

        self.pan_position = 0.0
        self.target_state = "SEARCHING"
        self.pan_step = 0.02
        self.lost_frames_count = 0
        self.max_lost_frames = 10

        self.current_pan_joint_position = 0.0
        self.locked_pan_position = 0.0
        self.candidate_center_pan = 0.0
        self.candidate_half_range = math.radians(60.0)
        self.focused_half_range = math.radians(30.0)
        self.candidate_direction = 1
        self.pan_joint_name = "pt_base_link_to_pt_link1"

        self.last_error_x = 0.0
        self.search_direction = 1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_yaw_global = None

        # ZEGAR WYWOŁUJĄCY METODE
        self.state_update = self.create_timer(0.05, self.machine_states)

    # CALLBACKS

    def weak_flag_callback(self, msg):
        self.weak_flag = msg.data

    def strong_flag_callback(self, msg):
        self.strong_flag = msg.data

    def joint_states_callback(self, msg):
        if self.pan_joint_name in msg.name:
            idx = msg.name.index(self.pan_joint_name)
            self.current_pan_joint_position = msg.position[idx]

    def error_callback(self, msg):
        self.last_error_x = msg.x

    def state_callback(self, msg):
        self.previous_target_state = self.target_state
        self.target_state = msg.data

        if self.target_state in [
            "CANDIDATE",
            "FOCUSED",
        ] and self.previous_target_state not in ["CANDIDATE", "FOCUSED"]:
            self.candidate_center_pan = self.current_pan_joint_position
            self.pan_position = self.current_pan_joint_position
            self.candidate_direction = 1 if self.last_error_x >= 0 else -1

        if (
            self.target_state == "CONFIRMED"
            and self.previous_target_state != "CONFIRMED"
        ):
            self.locked_pan_position = self.current_pan_joint_position

        if self.target_state == "SEARCHING":
            self.target_yaw_global = None
            self.candidate_center_pan = 0.0

    # STATES
    ## STATE TREE

    def machine_states(self):

        if self.target_state == "SEARCHING":
            self.searching_behavior()
        elif self.target_state == "CANDIDATE":
            self.candidate_behavior()
        elif self.target_state == "FOCUSED":
            # self.focused_behavior()
            self.candidate_behavior()
        elif self.target_state == "CONFIRMED":
            # self.confirmed_behavior()
            self.candidate_behavior()

    def searching_behavior(self):
        self.get_logger().info("State: SEARCHING")

        if not self.weak_flag:
            self.pan_position += self.pan_step * self.search_direction
            if self.pan_position > math.pi:
                self.pan_position = math.pi
                self.search_direction = -1
            elif self.pan_position < -math.pi:
                self.pan_position = -math.pi
                self.search_direction = 1
        else:
            self.get_logger().info(
                "Ruch podejrzany w SEARCHING! Lufa zamrożona - oczekiwanie na wyrok 5 klatek z YOLO..."
            )

        self.get_logger().info(
            f"Pan position: {self.pan_position}, pan step: {self.pan_step}"
        )
        self.send_data(self.pan_position)

    def candidate_behavior(self):
        self.get_logger().info("State: CANDIDATE (Global Stabilized + Search)")

        robot_yaw = self.get_robot_yaw()
        if robot_yaw is None:
            return

        # 1. Inicjalizacja przy pierwszym wykryciu
        if self.target_yaw_global is None:
            self.target_yaw_global = robot_yaw + self.current_pan_joint_position
            self.get_logger().info("Lock-on! Azymut celu zapisany.")

        # 2. Logika śledzenia i przeszukiwania
        if self.weak_flag:
            self.lost_frames_count = 0
            # Precyzyjne dociąganie azymutu na mapie
            pan_kp = 0.0008
            if abs(self.last_error_x) > 50.0:
                self.target_yaw_global += -self.last_error_x * pan_kp
        else:
            self.lost_frames_count += 1
            # Jeśli straciliśmy cel na dłużej - zrób mały sweep wokół ostatniej pozycji
            if self.lost_frames_count > self.max_lost_frames:
                sweep_amplitude = math.radians(30.0)
                sweep_offset = sweep_amplitude * math.sin(
                    self.get_clock().now().nanoseconds / 1e9 * 2.0
                )
                current_search_yaw = self.target_yaw_global + sweep_offset
            else:
                current_search_yaw = self.target_yaw_global

        # 3. Stabilizacja i publikacja
        if not self.weak_flag and self.lost_frames_count > self.max_lost_frames:
            self.pan_position = current_search_yaw - robot_yaw
        else:
            self.pan_position = self.target_yaw_global - robot_yaw

        self.pan_position = (self.pan_position + math.pi) % (2 * math.pi) - math.pi
        self.send_data(self.pan_position)

    def confirmed_behavior(self):
        self.get_logger().info("State: CONFIRMED")
        self.send_data(self.locked_pan_position)

    def focused_behavior(self):
        self.get_logger().info("State: FOCUSED")
        if self.weak_flag:
            self.lost_frames_count = 0
            pan_kp = 0.0005
            pan_correction = -self.last_error_x * pan_kp
            max_step = 0.05
            pan_correction = max(-max_step, min(max_step, pan_correction))
            self.pan_position += pan_correction
            self.candidate_center_pan = self.pan_position
        else:
            self.lost_frames_count += 1
            if self.lost_frames_count > self.max_lost_frames:
                min_pan = max(-math.pi, self.candidate_center_pan - self.focused_half_range)
                max_pan = min(math.pi, self.candidate_center_pan + self.focused_half_range)
                self.pan_position += self.pan_step * 0.15 * self.candidate_direction
                if self.pan_position > max_pan:
                    self.pan_position = max_pan
                    self.candidate_direction = -1
                elif self.pan_position < min_pan:
                    self.pan_position = min_pan
                    self.candidate_direction = 1
        self.send_data(self.pan_position)

    def send_data(self, position):
        msg = Float64()
        msg.data = position
        self.pan_pub.publish(msg)

    def get_robot_yaw(self):
        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", now)
            q = transform.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w])
            _, _, yaw = rot.as_euler("xyz")
            return yaw
        except Exception as e:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
