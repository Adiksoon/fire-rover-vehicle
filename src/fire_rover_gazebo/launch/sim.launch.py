from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="lidar_test.world",
        description="World file located in the fire_rover_gazebo/worlds directory.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lidar_test_world",
        description="Name of the SDF world to spawn the robot into.",
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare("fire_rover_gazebo"), "worlds", LaunchConfiguration("world")]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("ugv_description"), "urdf", "ugv_beast.urdf"]
    )

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--verbose", "-r", world_file],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
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

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot, bridge],
    )

    return LaunchDescription(
        [
            world_arg,
            world_name_arg,
            gazebo,
            delayed_spawn,
        ]
    )
