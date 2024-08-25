import rclpy
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="odom"
    )

    odom_topic = LaunchConfiguration("odom_topic")

    tracker = GroupAction(
        actions = [
            Node(
                package="bumperbot_utils",
                executable="bumperbot_tracker.py",
                arguments=[{"odom_topic": odom_topic}],
            ),
        ]
    )

    return LaunchDescription(
        [
            odom_topic_arg,
            tracker,
        ]
    )