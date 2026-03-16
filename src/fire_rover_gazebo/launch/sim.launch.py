from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Ścieżka do world file
    world_file = os.path.join(
        FindPackageShare("fire_rover_gazebo").find("fire_rover_gazebo"),
        "worlds",
        "empty.world",
    )

    # Ścieżka do URDF
    urdf_path = os.path.join(
        FindPackageShare("ugv_description").find("ugv_description"),
        "urdf",
        "ugv_beast.urdf",
    )

    # 1. Uruchomienie Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--verbose", world_file],
        output="screen",
    )

    # 2. Spawn robota do Ignition
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ugv_beast",
            "-file",
            urdf_path,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # 3. Bridge topików z Ignition do ROS2
    # Mapuj lidar_scan z Ignition na ROS2 topic
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            spawn_robot,
            bridge,
        ]
    )
