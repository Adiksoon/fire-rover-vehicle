import os

os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
os.environ["QT_QPA_PLATFORM"] = "xcb"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    set_sim_time = SetParameter(name="use_sim_time", value=True)

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
            "1.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["cat ", urdf_path]), value_type=str
                )
            }
        ],
    )

    joint_state_bridge_arg = "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            joint_state_bridge_arg,
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/pan_tilt/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/pan_tilt/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        output="screen",
    )
    ekf_config = PathJoinSubstitution(
        [FindPackageShare("fire_rover_gazebo"), "config", "ekf.yaml"]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    slam_config = PathJoinSubstitution(
        [FindPackageShare("fire_rover_gazebo"), "config", "slam.yaml"]
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config],
    )

    pan_controller = Node(
        package="fire_rover_perception",
        executable="pan_tilt_controller_node",
        name="pan_tilt_controller_node",
        output="screen",
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot, bridge],
    )

    delayed_ekf_slam = TimerAction(
        period=15.0,
        actions=[ekf_node, slam_node, pan_controller],
    )


    return LaunchDescription(
        [
            set_sim_time,
            world_arg,
            world_name_arg,
            gazebo,
            robot_state_publisher,
            delayed_spawn,
            delayed_ekf_slam,
        ]
    )
