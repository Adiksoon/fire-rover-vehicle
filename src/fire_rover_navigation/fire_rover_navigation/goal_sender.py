import math
import rclpy
import numpy as np
import scipy.ndimage as ndimage
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from tf_transformations import quaternion_from_euler
from time import time
from collections import deque
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

class GoalSender(Node):

    def __init__(self):
        super().__init__("goal_sender")

        # tworzymy klienta action
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # subskrybujemy mapę
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, "/frontiers_markers", 10)

        # czekamy na serwer action
        self.get_logger().info("Czekam na serwer action... ")
        self._action_client.wait_for_server()

        # inicjalizacja stanu robota
        self.current_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_call = None
        self.exploration_timer = self.create_timer(2.0,self.exploration_loop)

    def map_callback(self,msg):
        self.latest_map=msg

    def exploration_loop(self):


        if self.latest_map is None:
            self.get_logger().warn("Brak mapy")
            return


        # Pobieramy aktualną pozycję robota + TF map -> base_footprint
        msg=self.latest_map
        self.mapa_data=msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception as e:
            self.get_logger().warn(f"Nie można pobrać transformacji: {e}")
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution

        if self.current_goal is not None:
            dist_to_goal = math.hypot(self.current_goal[0] - robot_x, self.current_goal[1] - robot_y)

            if dist_to_goal > 1.0:
                return


        grid=np.array(data, dtype=np.int8).reshape((height, width))

        free_space=(grid == 0)
        unknown_space=(grid == -1)
        obstacles=(grid == 100)

        unknown_expanded=ndimage.binary_dilation(unknown_space)
        obstacles_expanded=ndimage.binary_dilation(obstacles, iterations=4)
        frontiers_mask=free_space & unknown_expanded & ~obstacles_expanded

        labeled_frontiers, num_clusters = ndimage.label(frontiers_mask)

        if num_clusters == 0:
            self.get_logger().info("Brak frontiera")
            return

        cluster_sizes=ndimage.sum(frontiers_mask, labeled_frontiers, range(1,num_clusters+1))

        valid_cluster_ids=[i+1 for i, size in enumerate(cluster_sizes) if size >= 20]

        if not valid_cluster_ids:
            self.get_logger().info("Brak frontiera")
            return

        centroids=ndimage.center_of_mass(frontiers_mask, labeled_frontiers, valid_cluster_ids)

        best_score = float("-inf")
        best_world_x = None
        best_world_y = None

        for idx, (cy, cx) in enumerate(centroids):
                #Konwersja GRID -> ŚWIAT
                world_x = origin_x + cx * resolution
                world_y = origin_y + cy * resolution

                #Obliczanie odległości od robota
                dist_to_robot = math.hypot(world_x - robot_x, world_y - robot_y)

                if dist_to_robot < 1.0:
                    continue

                c_id = valid_cluster_ids[idx]
                size = cluster_sizes[c_id-1]

                score=size-(dist_to_robot*0.5)

                if score > best_score:
                    best_score = score
                    best_world_x = world_x
                    best_world_y = world_y

        if best_world_x is None:
            self.get_logger().info("Frontiery sa zbyt blisko")
            return

        world_x=best_world_x
        world_y=best_world_y

        self.get_logger().info(f"Wybrano frontiera: ({world_x:.2f}, {world_y:.2f})")

        # Publikacja markerów w RViz
        self.publish_markers(centroids, origin_x, origin_y, resolution, world_x, world_y)

        goal_msg=NavigateToPose.Goal()
        goal_msg.pose.header.frame_id="map"
        goal_msg.pose.header.stamp=self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x=world_x
        goal_msg.pose.pose.position.y=world_y

        yaw = math.atan2(world_y - robot_y, world_x - robot_x)
        q = R.from_euler('z', yaw).as_quat()
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]




        self.current_goal=(world_x, world_y)
        self.result_future=self._action_client.send_goal_async(goal_msg)
        self.result_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal odrzucony ❌")
            self.current_goal = None
            return

        self.get_logger().info("Goal zaakceptowany ✅")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Hura! Dotarłem do celu! 🎯")

        self.current_goal = None

    def publish_markers(self, centroids, origin_x, origin_y, resolution, best_x, best_y):
        marker_array = MarkerArray()
        
        # 1. Czyszczenie starych markerów
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 2. Rysowanie wszystkich klastrów
        for i, (cy, cx) in enumerate(centroids):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "all_frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker.pose.position.x = origin_x + cx * resolution
            marker.pose.position.y = origin_y + cy * resolution
            marker.pose.position.z = 0.0
            
            marker_array.markers.append(marker)
            
        # 3. Zwycięzca - zielona sfera
        best_marker = Marker()
        best_marker.header.frame_id = "map"
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = "best_frontier"
        best_marker.id = 9999
        best_marker.type = Marker.SPHERE
        best_marker.action = Marker.ADD
        
        best_marker.scale.x = 0.5
        best_marker.scale.y = 0.5
        best_marker.scale.z = 0.5
        
        best_marker.color.r = 0.0
        best_marker.color.g = 1.0
        best_marker.color.b = 0.0
        best_marker.color.a = 0.9
        
        best_marker.pose.position.x = best_x
        best_marker.pose.position.y = best_y
        best_marker.pose.position.z = 0.1
        
        marker_array.markers.append(best_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
