import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import scipy.ndimage as ndimage
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState


class GoalSender(Node):

    def __init__(self):
        super().__init__("goal_sender")

        # TWORZENIE KLIENTA ACTION
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._action_client_spin = ActionClient(self, Spin, "/spin")

        # SUBSKRYBENCI
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        self.strong_flag_sub = self.create_subscription(
            Bool, "/search_detector/strong_target_flag", self.strong_flag_callback, 10
        )

        self.weak_flag_sub = self.create_subscription(
            Bool, "/search_detector/weak_target_flag", self.weak_flag_callback, 10
        )

        self.target_state_sub = self.create_subscription(
            String, "/search_detector/target_state", self.state_callback, 10
        )

        self.error_sub = self.create_subscription(
            Point, "/error_xy", self.error_callback, 10
        )

        # PUBLIKATORY
        self.marker_pub = self.create_publisher(MarkerArray, "/frontiers_markers", 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # INICJALIZACJA
        self.get_logger().info("Czekam na serwer action... ")
        self._action_client.wait_for_server()
        self._action_client_spin.wait_for_server()

        # INICJALIZACJA STANU ROBOTA
        self.current_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.strong_flag = False
        self.weak_flag = False
        self.active_goal_handle = None
        self.r_trig = RTrig()
        self.stopped = False
        self.error_x = 0.0

        self.target_state = "SEARCHING"
        self.previous_target_state = "SEARCHING"

        self.pan_joint_name = "pt_base_link_to_pt_link1"
        self.pan_joint_angle = 0.0
        self.candidate_reference_angle = 0.0
        self.alignment_active = False

        # Parametry dociągania bazy (Dead-zone i delay)
        self.alignment_delay = 1.0  # sekunda opóźnienia
        self.candidate_start_time = None
        self.dead_zone_start = 0.26  # ~15 stopni (start ruchu bazy)
        self.pan_joint_angle = 0.0
        self.candidate_reference_angle = 0.0

        # Parametry dociągania bazy (Safe Spin)
        self.is_spinning = False
        self.spin_trigger_threshold = 0.43  # ~25 stopni
        self.active_spin_handle = None
        self.candidate_start_time = None
        self.alignment_delay = 1.0  # 1 sekunda na potwierdzenie celu przed obrotem

        self.last_flag_time = None
        self.suspect_yaw = None

        self.alignment_delay = 1.5
        self.candidate_start_time = None
        self.is_spinning = False
        self.dead_zone_start = 0.26
        self.dead_zone_stop = 0.12

        # ZABEZPIECZENIE
        self.goal_uuid = 0
        self.blacklist = []  # lista krotek (x, y, timestamp)
        self.blacklist_ttl = 120.0  # sekundy życia wpisu na czarnej liście

        # TIMER
        self.state_update = self.create_timer(0.1, self.machine_states)

    # CALLBACKS

    def map_callback(self, msg):
        # pobieranie najnowszej iteracji mapy
        self.latest_map = msg

    def weak_flag_callback(self, msg):
        self.weak_flag = msg.data

    def strong_flag_callback(self, msg):

        self.strong_flag = msg.data

        if self.r_trig.update(self.strong_flag):
            if self.active_goal_handle:
                self.active_goal_handle.cancel_goal_async()
            self.current_goal = None
            self.stopped = False

    def state_callback(self, msg):
        self.previous_target_state = self.target_state
        self.target_state = msg.data

        # Logika czasu wykrycia i resetu akcji
        if self.target_state in [
            "CANDIDATE",
            "FOCUSED",
        ] and self.previous_target_state not in ["CANDIDATE", "FOCUSED"]:
            self.candidate_start_time = self.get_clock().now()

        if self.target_state not in ["CANDIDATE", "FOCUSED"]:
            self.candidate_start_time = None
            self.cancel_spin_goal()

        self.alignment_active = (
            True if self.target_state in ["CANDIDATE", "FOCUSED"] else False
        )

    def error_callback(self, msg):

        self.error_x = msg.x
        self.error_y = msg.y

    def joint_states_callback(self, msg):
        if self.pan_joint_name in msg.name:
            idx = msg.name.index(self.pan_joint_name)
            self.pan_joint_angle = -msg.position[idx]

    # MACHINE STATES
    def machine_states(self):
        self.get_logger().debug(f"Stan: {self.alignment_active}")

        if self.target_state in ["CANDIDATE", "FOCUSED"]:

            self.cancel_nav2_goal()

            # if self.alignment_active:
            #     self.align_to_candidate_target()
            # else:
            #     self.stop()
            #     self.get_logger().info(
            #         "Zatrzymuję się; baza wózka zrównana prostopadle do c#elu!  🎯"
            #     )

        elif self.target_state == "CONFIRMED":
            self.stop()

        elif self.target_state == "SEARCHING":
            self.cancel_spin_goal()
            if self.weak_flag or self.strong_flag:
                self.cancel_nav2_goal()

                self.last_flag_time = self.get_clock().now()

                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map", "base_footprint", Time()
                    )
                    q = transform.transform.rotation
                    r_rot = R.from_quat([q.x, q.y, q.z, q.w])
                    _, _, robot_yaw = r_rot.as_euler("xyz")

                    self.suspect_yaw = robot_yaw + self.pan_joint_angle
                except Exception as e:
                    self.get_logger().warn(f"TF wybuchło podczas szukania azymutu: {e}")
                    self.suspect_yaw = None

                self.get_logger().info(
                    "Zarys w SEARCHING! Wstrzymuję napęd, podglądam TF kompasu..."
                )
            else:
                self.exploration_loop()

    # STATES

    def align_to_candidate_target(self):
        self.get_logger().info("Stan: ALIGNING TO CANDIDATE/FOCUSED TARGET")

        if not self.alignment_active or self.candidate_start_time is None:
            self.get_logger().info(
                "Nie można wyrównać - brak aktywnego celu lub czasu startu."
            )
            return

        # Jeśli już się kręcimy, czekamy na wynik
        if self.is_spinning:
            self.get_logger().info("Oczekuję na wynik obrotu Spin...")
            return

        # Sprawdź opóźnienie przed pierwszym ruchem
        elapsed = (self.get_clock().now() - self.candidate_start_time).nanoseconds / 1e9
        if elapsed < self.alignment_delay:
            self.get_logger().info("czas przed ruchem")
            return

        angle_error = self.pan_joint_angle

        # Obrót tylko jeśli głowica jest mocno wychylona od osi wózka
        if abs(angle_error) > self.spin_trigger_threshold:
            self.get_logger().info(
                f"Wysyłam cel Spin: {math.degrees(angle_error):.1f} stopni"
            )
            self.send_spin_goal(angle_error)
            self.stopped = False

    def send_spin_goal(self, angle):
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = float(angle)

        self.is_spinning = True

        send_goal_future = self._action_client_spin.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.spin_response_callback)

    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Cel Spin odrzucony przez Nav2")
            self.is_spinning = False
            return

        self.active_spin_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        self.get_logger().info("Nav2 ukończył bezpieczny obrót bazy.")
        self.is_spinning = False
        self.active_spin_handle = None

    def cancel_spin_goal(self):
        if self.active_spin_handle:
            self.get_logger().info("Anuluję trwający obrót Spin")
            self.active_spin_handle.cancel_goal_async()
            self.active_spin_handle = None
            self.is_spinning = False

    def center_target(self, x):

        if abs(self.error_x) > 50:
            twist_msg = Twist()
            twist_msg.angular.z = -0.005 * x
            self.cmd_vel_pub.publish(twist_msg)

    def stop(self):

        if not self.stopped:

            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.stopped = True

    def cancel_nav2_goal(self):
        if self.active_goal_handle:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
            self.current_goal = None
            self.get_logger().info("Anuluję trasę z NAV2")

    def exploration_loop(self):
        self.stopped = False

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
            dist_to_goal = math.hypot(
                self.current_goal[0] - robot_x, self.current_goal[1] - robot_y  # type: ignore
            )
            if dist_to_goal > 1.0:
                return

        # KROK 3: Wyliczenie wieku ostatniego "ducha" z YOLO
        time_since_last_flag = float("inf")
        if self.last_flag_time is not None:
            time_nanos = (
                self.get_clock().now().nanoseconds - self.last_flag_time.nanoseconds
            )
            time_since_last_flag = time_nanos / 1e9

        # re-strukturyzacja 1D mapy ROS Ocupancy do klasycznej formy macierzy matematycznej NumPy
        grid = np.array(data, dtype=np.int8).reshape((height, width))

        # definiowanie fizycznych masek
        free_space = grid == 0
        unknown_space = grid == -1
        obstacles = grid == 100

        # DYLATACJA: Ochronne "nadmuchiwanie" nieznanych rogów i strefy zderzeniowej ścian (promień wyznaczony w symulatorze poprzez iterations)
        unknown_expanded = ndimage.binary_dilation(unknown_space)
        obstacles_expanded = ndimage.binary_dilation(obstacles, iterations=12)

        # wycięcie ostatecznych Krawędzi Półmroku (frontiers) matematycznie wykluczając strefy zablokowane
        frontiers_mask = free_space & unknown_expanded & ~obstacles_expanded  # type: ignore

        # zlecenie odnalezienia klastrów
        labeled_frontiers, num_clusters = ndimage.label(frontiers_mask)  # type: ignore

        if num_clusters == 0:
            self.get_logger().info(
                "Brak frontiera - misja badawcza zakończona (albo pokój staje się więzieniem)  🛑"
            )
            return

        # sumaryczne podliczenie rozmiarów (pikseli w długości powłoki) wszystkich powiązanych klastrów
        cluster_sizes = ndimage.sum(
            frontiers_mask, labeled_frontiers, range(1, num_clusters + 1)
        )

        # filtracja zakłóceń z liDara - szukaj dziur wielkości pow. dwudziestu pikseli
        valid_cluster_ids = [
            i + 1 for i, size in enumerate(cluster_sizes) if size >= 20
        ]

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

        centroid_scores = []

        for idx, (cy, cx) in enumerate(centroids):
            # konwersja cyfr Indexowych macierzy prosto pod miary systemu Światowego TF (metryczne)
            world_x = origin_x + cx * resolution
            world_y = origin_y + cy * resolution

            # BLACKLISTA: Jeśli cel znajduje się w skażonej strefie (blisko odrzuconych celów) pomijamy matematykę
            now = self.get_clock().now().nanoseconds / 1e9
            is_toxic = False
            for bx, by, bt in self.blacklist:
                if (now - bt) > self.blacklist_ttl:
                    continue  # wpis wygasł
                if math.hypot(world_x - bx, world_y - by) < 0.5:
                    is_toxic = True
                    break

            if is_toxic:
                centroid_scores.append(0.0)
                continue

            # odległość matematyczna Centroida bezpośrednio pod kołami
            dist_to_robot = math.hypot(world_x - robot_x, world_y - robot_y)

            # zakrawędziowanie "żeby nie wariował pod kołami" - rzucanie prosto w próżnię dalej niż my sami
            if dist_to_robot < 1.0:
                centroid_scores.append(0.0)
                continue

            # SYSTEM OCENY CELA: Klaster duży dostaje punkty, dystans podniesiony do kwadratu miażdży wyniki, by faworyzować najbliższe terytorium do roboty
            c_id = valid_cluster_ids[idx]
            size = cluster_sizes[c_id - 1]
            score = size / (dist_to_robot**2)

            # --- KROK 3: DIRECTIONAL BOOSTER ---
            # Jeśli widzieliśmy ducha wciągu ostatnich 15 sekund i mamy zapisany jego azymut na mapie
            if time_since_last_flag < 15.0 and self.suspect_yaw is not None:
                yaw_to_centroid = math.atan2(world_y - robot_y, world_x - robot_x)

                # Oblicz najkrótszą matematyczną odległość kątową po kole (uniknięcie błędu przeskoczenia z -Pi na Pi)
                angle_diff = abs(
                    (yaw_to_centroid - self.suspect_yaw + math.pi) % (2.0 * math.pi)
                    - math.pi
                )

                # Jeżeli cel leży w szerokim stożku poszukiwań (+/- 45 stopni czyli ~0.785 rad) z nosa lufy:
                if angle_diff < 0.8:
                    score *= (
                        8.0  # Ośmiokrotny potężny Boost nagrody! (Złota Gałąź Drzewa)
                    )

            centroid_scores.append(score)

            if score > best_score:
                best_score = score
                best_world_x = world_x
                best_world_y = world_y

        # blokada błędów logicznych, gdy żaden z dystansowych celów się po pętli NIE nadda
        if best_world_x is None:
            self.get_logger().info(
                "Frontiery uśmiercone z braku racjonalnej odległości lub w skutek wpisania do Czarnej Listy"
            )
            return

        world_x = best_world_x
        world_y = best_world_y

        self.get_logger().info(
            f"Wybrano idealnego Frontiera (Nagroda max): ({world_x:.2f}, {world_y:.2f})"
        )

        # publikacja zsynchronizowanego układu markerów dla okienka RViz
        self.publish_markers(
            centroids, centroid_scores, origin_x, origin_y, resolution, world_x, world_y
        )

        # kompilacja zlecenia nawigacji preempcyjnej do sterownika Nav2_Client
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y

        # pozycjonowanie wózka prostopadle na wyliczony horyzont
        yaw = math.atan2(world_y - robot_y, world_x - robot_x)  # type: ignore
        q = R.from_euler("z", yaw).as_quat()
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # zabezpieczanie przed duplikatami Timerów / Ostrzeżenia na preempcje
        self.current_goal = (world_x, world_y)
        self.goal_uuid += 1
        current_id = self.goal_uuid
        goal_coords = (world_x, world_y)  # zamknięcie koordynatów w closure

        self.result_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.result_future.add_done_callback(
            lambda future: self.goal_response_callback(future, current_id, goal_coords)
        )

    def goal_response_callback(self, future, current_id, goal_coords):
        # odpowiedź z sieci serwera - odbiór kurierski od Nav2
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal odrzucony u Kuriera w połowie lotu ❌")
            self.active_goal_handle = None

            # Wpisujemy cel natychmiast na Czarną Listę (koordynaty z closure, nie z self!)
            self.blacklist.append(
                (*goal_coords, self.get_clock().now().nanoseconds / 1e9)
            )

            # Upewniamy się, czy w międzyczasie Timer Pythona nie przysłał tu sam nowej ścieżki!
            if self.goal_uuid == current_id:
                self.current_goal = None
            return

        self.get_logger().info("Goal zaakceptowany przez mózg Nav2 ✅")
        self.active_goal_handle = goal_handle

        self.result_future = goal_handle.get_result_async()

        # po zaakceptowaniu, wieszamy asynchroniczną pętlę wyrokową...
        self.result_future.add_done_callback(
            lambda future: self.get_result_callback(future, current_id, goal_coords)
        )

    def get_result_callback(self, future, current_id, goal_coords):
        status = future.result().status

        # jeżeli wjechałeś poprawnie z statusem SUCCEEDED - cel osiągnięty!
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Hura! Dotarłem do celu! Frontiery odkryto. 🎯")
        elif status == GoalStatus.STATUS_CANCELED:
            # Anulowanie to decyzja świadoma (np. YOLO wykrył cel) — nie karz za to frontiera
            self.get_logger().info(
                "Trasa anulowana świadomie (cel wykryty?) — nie blacklistuję."
            )
        else:
            # jeżeli zdarzył się dramat w trakcie jazdy (wspomniany słynny Status 6) wózek wyrzuci błąd Nav2
            self.get_logger().warn(
                f"!!! TRASA ZERWANA W TRAKCIE !!! Nav2 status: {status}"
            )
            self.blacklist.append(
                (*goal_coords, self.get_clock().now().nanoseconds / 1e9)
            )

        # OCHRONA PRZED PREEMPCJĄ
        if current_id == self.goal_uuid:
            self.current_goal = None
            self.active_goal_handle = None

    def feedback_callback(self, feedback_msg):
        pass

    def publish_markers(
        self, centroids, centroid_scores, origin_x, origin_y, resolution, best_x, best_y
    ):
        marker_array = MarkerArray()

        # 1. DELETE - Czyszczenie z monitorów starych, bezużytecznych już punktów (wycieraczka)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Matematyka normalizująca dla wizualizacji (chroni przed absurdalnymi rozmiarami po zboostowaniu)
        max_s = 1.0
        min_s = 0.0
        if centroid_scores:
            max_val = max(centroid_scores)
            min_val = min(centroid_scores)
            if max_val > min_val:
                max_s = max_val
                min_s = min_val

        # 2. Rysowanie masowo wszystkich rozpatrywanych matematycznie przez kod centroidów klastra
        for i, (cy, cx) in enumerate(centroids):
            score = centroid_scores[i] if i < len(centroid_scores) else 0.0

            # Wciśnij wszystkie punkty nagród do skalera [od 0.1 metra do chamskiego 0.6 metra] w okienku!
            norm_size = 0.1
            if max_s > min_s:
                norm_size = 0.1 + 0.5 * ((score - min_s) / (max_s - min_s))

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "all_frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = norm_size
            marker.scale.y = norm_size
            marker.scale.z = norm_size

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6

            marker.pose.position.x = origin_x + cx * resolution
            marker.pose.position.y = origin_y + cy * resolution
            marker.pose.position.z = 0.0

            marker_array.markers.append(marker)  # type:ignore

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

        marker_array.markers.append(best_marker)  # type:ignore

        self.marker_pub.publish(marker_array)


class RTrig:
    def __init__(self):
        self.prev = False

    def update(self, signal):
        triggered = not self.prev and signal
        self.prev = signal
        return triggered


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
