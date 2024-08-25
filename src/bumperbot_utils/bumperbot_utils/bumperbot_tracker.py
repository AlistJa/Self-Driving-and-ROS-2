#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class BumperbotTracker(Node):
    def __init__(self):
        super().__init__("bumperbot_tracker")

        self.declare_parameter("odom_topic", "odom")

        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.odom_sub_ = self.create_subscription(Odometry, "/bumperbot_controller/odom", self.odomCallback, 10)

        self.path_pub_ = self.create_publisher(Path, "/bumperbot_controller/trajectory", 10)

    def odomCallback(self, msg):
        #self.get_logger().info(f"Received odometry message {msg.pose}")
        #self.path_msg_.header.stamp = msg.header.stamp
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = self.get_parameter("odom_topic").get_parameter_value().string_value
        poseStamped.pose.position.x = msg.pose.pose.position.x
        poseStamped.pose.position.y = msg.pose.pose.position.y
        poseStamped.pose.position.z = msg.pose.pose.position.z

        poseStamped.pose.orientation.x = msg.pose.pose.orientation.x
        poseStamped.pose.orientation.y = msg.pose.pose.orientation.y
        poseStamped.pose.orientation.z = msg.pose.pose.orientation.z
        poseStamped.pose.orientation.w = msg.pose.pose.orientation.w

        self.path_msg_.poses.append(poseStamped)

        self.path_pub_.publish(self.path_msg_)

def main():
    rclpy.init()
    bumperbot_tracker = BumperbotTracker()
    rclpy.spin(bumperbot_tracker)
    bumperbot_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()