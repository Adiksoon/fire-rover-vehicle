import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from math import cos, sin, isinf, isnan
import numpy as np


class SLAMDebugNode(Node):

    def __init__(self):
        super().__init__("slam_debug_node")
        self.declare_parameter("use_sim_time", True)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan, "/lidar_scan", self.scan_callback, qos_profile_sensor_data
        )
        self.grid_publisher = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.resolution = 0.05
        self.grid_size = 200

        # Domyślna wartość w ROS to -1 (nieznana przestrzeń) i typ int8
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        self.publish_map_to_odom_tf()
        self.timer = self.create_timer(2.0, self.publish_grid)

    def publish_map_to_odom_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(msg)

    def scan_callback(self, msg):
        angle = msg.angle_min
        offset = self.grid_size // 2

        for r in msg.ranges:
            if isinf(r) or isnan(r):
                angle += msg.angle_increment
                continue

            x = r * cos(angle)
            y = r * sin(angle)

            # Obliczanie indeksów z uwzględnieniem offsetu
            ix = int(x / self.resolution) + offset
            iy = int(y / self.resolution) + offset

            # Aktualizacja mapy tylko w granicach tablicy
            if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                self.grid[iy, ix] = 100

            angle += msg.angle_increment

        self.get_logger().info("Mapa zaktualizowana")

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size

        # Ustawienie origin na środek (w metrach)
        msg.info.origin.position.x = -(self.grid_size * self.resolution) / 2
        msg.info.origin.position.y = -(self.grid_size * self.resolution) / 2
        msg.info.origin.orientation.w = 1.0

        # Rzutowanie na listę z tablicy 1D w int8
        msg.data = self.grid.flatten().tolist()

        self.grid_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMDebugNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
