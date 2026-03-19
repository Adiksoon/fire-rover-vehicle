import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):

    def __init__(self):
        super().__init__("goal_sender")

        # tworzymy klienta action
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # wywołujemy wysłanie goal
        self.send_goal()

    def send_goal(self):
        # czekamy aż Nav2 będzie gotowy
        self.get_logger().info("Czekam na server...")
        self._action_client.wait_for_server()

        # tworzymy goal
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 2.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Wysyłam goal...")

        # wysyłamy goal (asynchronicznie)
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # callback gdy goal przyjęty
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal odrzucony ❌")
            return

        self.get_logger().info("Goal zaakceptowany ✅")

        # czekamy na wynik
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Dotarłem do celu 🎯")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
