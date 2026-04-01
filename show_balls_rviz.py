#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class BallsRvizPublisher(Node):
    def __init__(self):
        super().__init__('balls_rviz_publisher')
        # Główne pasmo do rzutowania danych z Gazebo na RViz
        self.pub = self.create_publisher(MarkerArray, '/gazebo_balls_visual', 10)
        self.timer = self.create_timer(1.0, self.publish_balls)
        
        # Wyciągnięte skrupulatnie pozycje statyczne startowe dla lidar_test.world
        self.balls = [
            ("ball_front_corner", 3.85, 4.27),
            ("ball_under_desk", -4.7, 0.5),
            ("ball_back_side", -0.5, -4.7),
        ]
        
        self.get_logger().info("Holo-rzutnik Prawdy Absolutnej z Gazebo włączony! Otwórz RViz2...")

    def publish_balls(self):
        msg = MarkerArray()
        
        for i, (name, x, y) in enumerate(self.balls):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "gazebo_balls_ground_truth"
            m.id = i
            
            # Reprezentacja fizycznej bryły (w RViz będzie to potężna, idealna kula)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.15 # Wzniesienie od podłogi żeby LiDAR tego nie zjadł w 2D

            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            
            # Wzorcowy Kolor pomarańczowo-żółty jak piłki RoboCup
            m.color.r = 1.0 
            m.color.g = 0.6
            m.color.b = 0.0
            m.color.a = 0.9
            
            msg.markers.append(m)
            
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = BallsRvizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
