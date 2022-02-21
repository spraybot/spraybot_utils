#!/usr/bin/env python3

import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class WaypointTeleop(Node):

    def __init__(self):
        super().__init__('waypoint_teleop')

        self.declare_parameters(
            '', [('waypoint_topic', '/goal_pose'), ('axis_scaling_factor', 0.25)]
        )

        self.sub_joy_ = self.create_subscription(
            Joy, '/joy_teleop/joy', self.sub_joy_cb, 10
        )
        self.sub_odom_ = self.create_subscription(
            Odometry, '/odometry/filtered', self.sub_odom_cb, 10
        )
        self.pub_waypoint_ = self.create_publisher(
            PoseStamped,
            self.get_parameter('waypoint_topic').get_parameter_value().string_value,
            10,
        )

        self.axis_scaling_factor_ = (
            self.get_parameter('axis_scaling_factor').get_parameter_value().double_value
        )
        self.reset_waypoint_ = False
        self.publish_waypoint_ = False

        self.waypoint_pos_ = PoseStamped()
        self.waypoint_pos_.header.frame_id = 'odom'
        self.waypoint_pos_.pose.position.x = 0.0
        self.waypoint_pos_.pose.position.y = 0.0

        self.axis_value = [0.0, 0.0]

        timer_period_ = 0.1
        self.timer_ = self.create_timer(timer_period_, self.timer_callback)

    def sub_joy_cb(self, msg):
        if msg.buttons[-1] == 1:
            self.reset_waypoint_ = True
        if msg.buttons[1] == 1:
            self.publish_waypoint_ = not self.publish_waypoint_
            self.get_logger().info(
                'Waypoint Teleop Status: ' + str(self.publish_waypoint_)
            )
            time.sleep(1)
            self.reset_waypoint_ = True

        self.axis_value = [msg.axes[4], msg.axes[3]]

    def sub_odom_cb(self, msg):
        if self.reset_waypoint_:
            self.waypoint_pos_.pose.position.x = msg.pose.pose.position.x
            self.waypoint_pos_.pose.position.y = msg.pose.pose.position.y
            self.reset_waypoint_ = False

    def timer_callback(self):
        if not self.publish_waypoint_:
            return
        self.waypoint_pos_.pose.position.x += (
            self.axis_value[0] * self.axis_scaling_factor_
        )
        self.waypoint_pos_.pose.position.y += (
            self.axis_value[1] * self.axis_scaling_factor_
        )
        self.pub_waypoint_.publish(self.waypoint_pos_)


def main(args=None):
    rclpy.init(args=args)

    waypoint_teleop = WaypointTeleop()

    rclpy.spin(waypoint_teleop)

    waypoint_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
