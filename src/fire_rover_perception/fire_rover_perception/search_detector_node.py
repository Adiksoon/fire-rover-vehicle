import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class SearchDetectorNode(Node):
    def __init__(self):
        super().__init__("search_detector_node")

        #SUBSKRYBCJE
        self.raw_sub=self.create_subscription(Image, "/camera/image_raw",self.raw_callback,10)
        self.photo_info_sub=self.create_subscription(Image, "/camera/camera_info", self.info_callback, 10)

        #Inicjalizacja mostu
        self.bridge=CvBridge()

        #Ładowanie modeli
        self.model=YOLO("yolov8n.pt")

    def raw_callback(self, msg):
        try:
            cv_image=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return

        results=self.model(cv_image, conf=0.2)

        if len(results)>0:
            result=results[0]

        boxes=result.plot()

        cv2.imshow("Detekcja", boxes)
        cv2.waitKey(1)

    def info_callback(self, msg):
        self.get_logger().info("Otrzymano informacje o obrazie")







def main(args=None):
    rclpy.init(args=args)
    node = SearchDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



