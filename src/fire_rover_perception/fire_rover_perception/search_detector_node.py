import os

os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String


class SearchDetectorNode(Node):
    def __init__(self):
        super().__init__("search_detector_node")

        # SUBSKRYBCJE
        self.raw_sub = self.create_subscription(
            Image, "/camera/image_raw", self.raw_callback, 10
        )
        self.photo_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, 10
        )

        # PUBLIKACJE
        self.error_pub = self.create_publisher(Point, "/error_xy", 10)
        self.found_weak_flag_pub = self.create_publisher(
            Bool, "/search_detector/weak_target_flag", 10
        )

        self.found_strong_flag_pub = self.create_publisher(
            Bool, "/search_detector/strong_target_flag", 10
        )

        self.state_pub = self.create_publisher(
            String, "/search_detector/target_state", 10
        )
        self.bridge = CvBridge()
        self.weak_flag = False
        self.strong_flag = False

        self.target_state = "SEARCHING"

        # Globalne liczniki ciągłych klatek (Hits & Misses)
        self.consecutive_hits = 0
        self.consecutive_misses = 0
        self.seen_strong = False

        self.strong_conf_threshold = 0.8
        self.weak_conf_threshold = 0.6

        self.search_confirm_threshold = 5     # Awans na kandydata (Odsiew Duchów)
        self.focus_confirm_threshold = 10     # Awans na Focusa
        self.candidate_missed_threshold = 75  # Spadek do szukania
        self.focused_missed_threshold = 30    # Utrata Focusa
        self.confirmed_missed_threshold = 150 # Utrata Confirmed

        self.model = YOLO("yolo26m.pt")

        # ZEGAR WYWOŁUJĄCY METODE
        self.state_update_timer = self.create_timer(0.1, self.state_update)
        self.detection_timer = self.create_timer(0.1, self.process_detection)

    # CALLBACKS

    def raw_callback(self, msg):
        self.latest_image = msg

    def info_callback(self, msg):
        pass

    # METHODS

    def process_detection(self):

        if not hasattr(self, "latest_image"):
            return

        error_x = 0.0
        error_y = 0.0
        self.weak_flag = False
        self.strong_flag = False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return

        # Piłka RoboCup jest modelem w pełni i prawdziwie teksturowanym, lapiemy go z łatwą pewnością PBR!
        results = self.model(cv_image, conf=self.weak_conf_threshold, verbose=False)

        result = results[0] if len(results) > 0 else None

        for box in result.boxes if result is not None else []:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            # Klasa 32 to czystej krwi Piłka Sportowa w bibliotekach uczenia maszynowego (Sports Ball / COCO dataset)
            if cls == 32:
                cx, cy, w, h = box.xywh[0]
                self.get_logger().info("Znaleziono słaby cel.")
                error_x = float(cx.item()) - 960.0

                self.weak_flag = True

                if conf >= self.strong_conf_threshold:
                    self.get_logger().info("Znaleziono silny cel.")
                    self.strong_flag = True

                break

        msg = Point()
        msg.x = error_x
        msg.y = error_y
        self.error_pub.publish(msg)

        msg_weak_flag = Bool()
        msg_weak_flag.data = self.weak_flag
        self.found_weak_flag_pub.publish(msg_weak_flag)

        msg_strong_flag = Bool()
        msg_strong_flag.data = self.strong_flag
        self.found_strong_flag_pub.publish(msg_strong_flag)

        if result is not None:
            cv2.imshow("Detekcja", result.plot())
            cv2.waitKey(1)

    def state_update(self):
        state_msg = String()

        if self.weak_flag:
            self.consecutive_hits += 1
            self.consecutive_misses = 0
        else:
            self.consecutive_misses += 1
            self.consecutive_hits = 0

        if self.target_state == "SEARCHING":
            if self.weak_flag or self.strong_flag:
                if self.consecutive_hits >= self.search_confirm_threshold:
                    self.target_state = "CANDIDATE"
                    self.consecutive_misses = 0

       elif self.target_state == "CANDIDATE":
            if self.strong_flag:
                self.seen_strong = True

            if self.consecutive_hits >= self.focus_confirm_threshold and self.seen_strong:
                self.target_state = "FOCUSED"
                self.consecutive_misses = 0
                self.seen_strong = False
            elif self.consecutive_misses >= self.candidate_missed_threshold:
                self.target_state = "SEARCHING"
                self.consecutive_misses = 0
                self.seen_strong = False

        elif self.target_state == "FOCUSED":
            if self.strong_flag:
                if self.consecutive_hits >= self.focus_confirm_threshold:
                    self.target_state = "CONFIRMED"
                    self.consecutive_misses = 0
            elif self.consecutive_misses >= self.focused_missed_threshold:
                self.target_state = "CANDIDATE"
                self.consecutive_misses = 0
                self.seen_strong = False

        elif self.target_state == "CONFIRMED":
            if self.consecutive_misses >= self.confirmed_missed_threshold:
                self.target_state = "CANDIDATE"
                self.consecutive_misses = 0
                self.seen_strong = False

        state_msg.data = self.target_state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SearchDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
