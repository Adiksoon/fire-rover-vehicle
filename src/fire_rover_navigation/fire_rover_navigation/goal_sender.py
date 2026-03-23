import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from tf_transformations import quaternion_from_euler
from time import time


class GoalSender(Node):

    def __init__(self):
        super().__init__("goal_sender")

        # tworzymy klienta action
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # subskrybujemy mapę
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # czekamy na serwer action
        self.get_logger().info("Czekam na serwer action... ")
        self._action_client.wait_for_server()

        # inicjalizacja stanu robota
        self.robot_is_busy = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_call = 0

    def map_callback(self, msg):

        # Ograniczamy częstotliwość wywoływania tej funkcji do 1 na 2 sekundy, żeby nie wysyłać zbyt wielu celów do Nav2
        if time() - self.last_call < 2:
            return
        self.last_call = time()

        # Sprawdzamy stan robota
        if self.robot_is_busy:
            return
        frontiers = []

        # Pobieramy aktualną pozycję robota + TF map -> base_footprint
        self.map_data = msg
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

        # Algorytm wykrywania frontierów
        for y in range(height):
            for x in range(width):
                index = y * width + x
                cell = data[index]
                if cell == 0:  # sprawdzamy tylko wolne komórki
                    # Sprawdzamy, czy dana wolna komórka jest frontierem tzn. sąsiąd ma wartość -1
                    if self.has_unknown_neighbor(x, y, data, width, height):
                        frontiers.append((x, y))

        if not frontiers:
            self.get_logger().info("Eksploracja zakonczona!")
            return

        # Wybieramy najlepszy frontier jako cel (najbliższy robotowi)
        best = None
        min_dist = float("inf")

        for point in frontiers:
            fx = point[0]
            fy = point[1]
            # Konwersja GRID -> ŚWIAT
            wx = msg.info.origin.position.x + (fx + 0.5) * msg.info.resolution
            wy = msg.info.origin.position.y + (fy + 0.5) * msg.info.resolution

            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist < min_dist and dist > 0.3:
                min_dist = dist
                best = (wx, wy)

        if best is None:
            self.get_logger().info("Nie można znaleźć frontiera!")
            return

        world_x, world_y = best

        # Obliczamy orientacje
        yaw = math.atan2(world_y - robot_y, world_x - robot_x)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        # Wysyłamy goal do Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.robot_is_busy = True

        self.result_future = self._action_client.send_goal_async(goal_msg)
        self.result_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal odrzucony ❌")
            self.robot_is_busy = False
            return

        self.get_logger().info("Goal zaakceptowany ✅")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Dotarłem do celu 🎯")
        self.robot_is_busy = False

    def has_unknown_neighbor(self, x, y, data, width, height):

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:

                nx = x + dx
                ny = y + dy

                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue

                index = ny * width + nx

                if data[index] == -1:
                    return True

        return False


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
