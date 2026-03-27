import os

os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class SearchDetectorNode(Node):
    def __init__(self):
        super().__init__("search_detector_node")

        # SUBSKRYBCJE
        self.raw_sub = self.create_subscription(
            Image, "/camera/image_raw", self.raw_callback, 10
        )
        self.photo_info_sub = self.create_subscription(
            Image, "/camera/camera_info", self.info_callback, 10
        )

        # PUBLIKACJE
        self.error_pub = self.create_publisher(Point, "/error_xy", 10)
        self.flag_pub = self.create_publisher(Bool, "/flag", 10)

        # INICJALIZACJE
        self.bridge = CvBridge()
        self.flag = False
        ##ładowanie modeli
        self.model = YOLO("yolo26m.pt")

    def raw_callback(self, msg):
        self.flag = False
        error_x = 0.0
        error_y = 0.0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return

        # Piłka RoboCup jest modelem w pełni i prawdziwie teksturowanym, lapiemy go z łatwą pewnością PBR!
        results = self.model(cv_image, conf=0.50, verbose=False)

        if len(results) > 0:
            result = results[0]
        else:
            return

        for box in result.boxes:
            cls = int(box.cls[0])

            # Klasa 32 to czystej krwi Piłka Sportowa w bibliotekach uczenia maszynowego (Sports Ball / COCO dataset)
            if cls == 32:
                cx, cy, w, h = box.xywh[0]

                # Nowy środek matrycy dla podbitej przez URDF rozdzielczości Full HD (1920 px szerokości)
                error_x = float(cx.item()) - 960.0
                self.flag = True

        msg = Point()
        msg.x = error_x
        msg.y = error_y
        self.error_pub.publish(msg)

        msg_flaga = Bool()
        msg_flaga.data = self.flag
        self.flag_pub.publish(msg_flaga)

        cv2.imshow("Detekcja", result.plot())
        cv2.waitKey(1)

    def info_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SearchDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
