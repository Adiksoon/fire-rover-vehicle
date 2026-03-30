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
        self.found_flag_pub = self.create_publisher(Bool, "/found_target_flag", 10)
        self.state_pub = self.create_publisher(
            String, "/search_detector/target_state", 10
        )
        # INICJALIZACJE
        self.bridge = CvBridge()
        self.found_flag = False

        self.target_state = "SEARCHING"
        self.previous_target_state = "SEARCHING"
        self.locked_pan_position = 0.0

        self.detected_count = 0
        self.missed_count = 0

        self.confirm_threshold = 5
        self.candidate_missed_threshold = 7
        self.lost_threshold = 50

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
        self.found_flag = False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return

        # Piłka RoboCup jest modelem w pełni i prawdziwie teksturowanym, lapiemy go z łatwą pewnością PBR!
        results = self.model(cv_image, conf=0.50, verbose=False)

        result = results[0] if len(results) > 0 else None

        for box in result.boxes if result is not None else []:
            cls = int(box.cls[0])

            # Klasa 32 to czystej krwi Piłka Sportowa w bibliotekach uczenia maszynowego (Sports Ball / COCO dataset)
            if cls == 32:
                cx, cy, w, h = box.xywh[0]
                self.get_logger().info(
                    f"Znaleziono piłkę! Środek: ({cx.item()}, {cy.item()})"
                )
                # Nowy środek matrycy dla podbitej przez URDF rozdzielczości Full HD (1920 px szerokości)
                error_x = float(cx.item()) - 960.0
                self.found_flag = True
                break

        msg = Point()
        msg.x = error_x
        msg.y = error_y
        self.error_pub.publish(msg)

        msg_flaga = Bool()
        msg_flaga.data = self.found_flag
        self.found_flag_pub.publish(msg_flaga)

        if result is not None:
            cv2.imshow("Detekcja", result.plot())
            cv2.waitKey(1)

    def state_update(self):
        state_msg = String()

        if self.found_flag:
            self.detected_count += 1
            self.missed_count = 0
        else:
            self.missed_count += 1
            self.detected_count = 0

        if self.target_state == "SEARCHING":
            if self.found_flag:
                self.target_state = "CANDIDATE"

        elif self.target_state == "CANDIDATE":
            if self.detected_count >= self.confirm_threshold:
                self.target_state = "CONFIRMED"
            elif self.missed_count >= self.candidate_missed_threshold:
                self.target_state = "SEARCHING"

        elif self.target_state == "CONFIRMED":
            if self.missed_count >= self.lost_threshold:
                self.target_state = "LOST"
        elif self.target_state == "LOST":
            if self.found_flag:
                self.target_state = "CANDIDATE"
                self.detected_count = 1
                self.missed_count = 0
            elif self.missed_count >= self.lost_threshold:
                self.target_state = "SEARCHING"

        state_msg.data = self.target_state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SearchDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
