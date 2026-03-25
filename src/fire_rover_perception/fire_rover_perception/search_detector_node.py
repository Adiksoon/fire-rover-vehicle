import rclpy
from rclpy.node import Node

class SearchDetectorNode(Node):
    def __init__(self):
        super().__init__("search_detector_node")

        #SUBSKRYBCJE
        self.raw_sub=create_subscription(Image, "/camera/image_raw",self.raw_callback,10)
        self.photo_info_sub=create_subscription(Image, "/camera/camera_info", info_callback, 10)








def main(args=None):
    rclpy.init(args=args)
    node = SearchDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



