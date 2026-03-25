import math
import rclpy
import numpy as np
import scipy.ndimage as ndimage
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

class GoalSender(Node):

    def __init__(self):
        super().__init__("goal_sender")

        # TWORZENIE KLIENTA ACTION
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # SUBSKRYBENCI
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # PUBLIKATORY
        self.marker_pub = self.create_publisher(MarkerArray, "/frontiers_markers", 10)

        # INICJALIZACJA
        self.get_logger().info("Czekam na serwer action... ")
        self._action_client.wait_for_server()

        # INICJALIZACJA STANU ROBOTA
        self.current_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None

        # INICJALIZACJA STANU ROBOTA
        self.current_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None

        # ZABEZPIECZENIE
        self.goal_uuid = 0
        self.blacklist = []

        # POWOŁANIE TIMERA
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)

    def map_callback(self, msg):
        # pobieranie najnowszej iteracji mapy
        self.latest_map = msg

    def exploration_loop(self):
        # awaryjne zrzucenie kalkulacji, o ile subskrybent nie dostarczył pierwszego obrazu z LiDARA
        if self.latest_map is None:
            self.get_logger().warn("Brak mapy")
            return

        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        # pobieranie aktualnej pozycji lokalnej robota: map -> base_footprint
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception as e:
            self.get_logger().warn(f"Nie można pobrać transformacji odom/map: {e}")
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution

        # blokada preempcji: Timer nie wrzuci nowego celu, jeśli fizyczna podróż do obecnego ma pow. 1.0 metra odległości
        if self.current_goal is not None:
            dist_to_goal = math.hypot(self.current_goal[0] - robot_x, self.current_goal[1] - robot_y)
            if dist_to_goal > 1.0:
                return

        # re-strukturyzacja 1D mapy ROS Ocupancy do klasycznej formy macierzy matematycznej NumPy
        grid = np.array(data, dtype=np.int8).reshape((height, width))

        # definiowanie fizycznych masek
        free_space = (grid == 0)
        unknown_space = (grid == -1)
        obstacles = (grid == 100)

        # DYLATACJA: Ochronne "nadmuchiwanie" nieznanych rogów i strefy zderzeniowej ścian (promień wyznaczony w symulatorze poprzez iterations)
        unknown_expanded = ndimage.binary_dilation(unknown_space)
        obstacles_expanded = ndimage.binary_dilation(obstacles, iterations=8)

        # wycięcie ostatecznych Krawędzi Półmroku (frontiers) matematycznie wykluczając strefy zablokowane
        frontiers_mask = free_space & unknown_expanded & ~obstacles_expanded

        # zlecenie odnalezienia klastrów
        labeled_frontiers, num_clusters = ndimage.label(frontiers_mask)

        if num_clusters == 0:
            self.get_logger().info("Brak frontiera - misja badawcza zakończona (albo pokój staje się więzieniem)  🛑")
            return

        # sumaryczne podliczenie rozmiarów (pikseli w długości powłoki) wszystkich powiązanych klastrów
        cluster_sizes = ndimage.sum(frontiers_mask, labeled_frontiers, range(1, num_clusters + 1))

        # filtracja zakłóceń z liDara - szukaj dziur wielkości pow. dwudziestu pikseli
        valid_cluster_ids = [i + 1 for i, size in enumerate(cluster_sizes) if size >= 20]

        if not valid_cluster_ids:
            self.get_logger().info("Brak solidnego frontiera (wyłącznie szum)")
            return

        # Wyznacznik punktów reprezentatytwnych (Zamiast rzucać Center_Of_Mass, bierzemy piksel Z LINII, by uniknąć środka w kształcie rogalika)
        centroids = []
        for c_id in valid_cluster_ids:
            # Wyciągnij koordynaty wszystkich pikseli nalezacych do tego jednego Klastra
            points = np.argwhere(labeled_frontiers == c_id)
            # Wybierz piksel leżący dokładnie "w połowie" namierzonej krawędzi:
            mid_idx = len(points) // 2
            centroids.append((points[mid_idx][0], points[mid_idx][1]))

        best_score = float("-inf")
        best_world_x = None
        best_world_y = None

        for idx, (cy, cx) in enumerate(centroids):
            # konwersja cyfr Indexowych macierzy prosto pod miary systemu Światowego TF (metryczne)
            world_x = origin_x + cx * resolution
            world_y = origin_y + cy * resolution

            # BLACKLISTA: Jeśli cel znajduje się w skażonej strefie (blisko odrzuconych celów) pomijamy matematykę
            is_toxic = False
            for bx, by in self.blacklist:
                if math.hypot(world_x - bx, world_y - by) < 0.5:
                    is_toxic = True
                    break

            if is_toxic:
                continue

            # odległość matematyczna Centroida bezpośrednio pod kołami
            dist_to_robot = math.hypot(world_x - robot_x, world_y - robot_y)

            # zakrawędziowanie "żeby nie wariował pod kołami" - rzucanie prosto w próżnię dalej niż my sami
            if dist_to_robot < 1.0:
                continue

            # SYSTEM OCENY CELA: Klaster duży dostaje punkty, dystans podniesiony do kwadratu miażdży wyniki, by faworyzować najbliższe terytorium do roboty
            c_id = valid_cluster_ids[idx]
            size = cluster_sizes[c_id - 1]
            score = size / (dist_to_robot ** 2)

            if score > best_score:
                best_score = score
                best_world_x = world_x
                best_world_y = world_y

        # blokada błędów logicznych, gdy żaden z dystansowych celów się po pętli NIE nadda
        if best_world_x is None:
            self.get_logger().info("Frontiery uśmiercone z braku racjonalnej odległości lub w skutek wpisania do Czarnej Listy")
            return

        world_x = best_world_x
        world_y = best_world_y

        self.get_logger().info(f"Wybrano idealnego Frontiera (Nagroda max): ({world_x:.2f}, {world_y:.2f})")

        # publikacja zsynchronizowanego układu markerów dla okienka RViz
        self.publish_markers(centroids, origin_x, origin_y, resolution, world_x, world_y)

        # kompilacja zlecenia nawigacji preempcyjnej do sterownika Nav2_Client
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y

        # pozycjonowanie wózka prostopadle na wyliczony horyzont
        yaw = math.atan2(world_y - robot_y, world_x - robot_x)
        q = R.from_euler('z', yaw).as_quat()
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # zabezpieczanie przed duplikatami Timerów / Ostrzeżenia na preempcje
        self.current_goal = (world_x, world_y)
        self.goal_uuid += 1
        current_id = self.goal_uuid

        self.result_future = self._action_client.send_goal_async(goal_msg)
        self.result_future.add_done_callback(lambda future: self.goal_response_callback(future, current_id))

    def goal_response_callback(self, future, current_id):
        # odpowiedź z sieci serwera - odbiór kurierski od Nav2
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal odrzucony u Kuriera w połowie lotu ❌")

            # Wpisujemy cel natychmiast na Czarną Listę by zapomnieć go na zawsze
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)

            # Upewniamy się, czy w międzyczasie Timer Pythona nie przysłał tu sam nowej ścieżki!
            if self.goal_uuid == current_id:
                self.current_goal = None
            return

        self.get_logger().info("Goal zaakceptowany przez mózg Nav2 ✅")
        self.result_future = goal_handle.get_result_async()

        # po zaakceptowaniu, wieszamy asynchroniczną pętlę wyrokową...
        self.result_future.add_done_callback(lambda future: self.get_result_callback(future, current_id))

    def get_result_callback(self, future, current_id):
        status = future.result().status

        # jeżeli wjechałeś poprawnie z statusem SUCCEEDED - cel osiągnięty!
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Hura! Dotarłem do celu! Frontiery odkryto. 🎯")
        else:
            # jeżeli zdarzył się dramat w trakcie jazdy (wspomniany słynny Status 6) wózek wyrzuci błąd Nav2. Oznaczamy skazany teren do Zeszytu Czarnej Lity. ☠️
            self.get_logger().warn(f"!!! TRASA ZERWANA W TRAKCIE !!! Nav2 wypluł się błędem (np. 6 = ABORTED): {status}")
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)

        # OCHRONA PRZED PREEMPCJĄ: Nawet jeśli wózek się rozbił ze statusem 6, wyczyścimy "obecny cel", Z WYJĄTKIEM sytuacji gdy zdążyliśmy wbić się w timer z Nowym Celem z ID!
        if current_id == self.goal_uuid:
            self.current_goal = None

    def publish_markers(self, centroids, origin_x, origin_y, resolution, best_x, best_y):
        marker_array = MarkerArray()

        # 1. DELETE - Czyszczenie z monitorów starych, bezużytecznych już punktów (wycieraczka)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 2. Rysowanie masowo wszystkich rozpatrywanych matematycznie przez kod centroidów klastra
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
