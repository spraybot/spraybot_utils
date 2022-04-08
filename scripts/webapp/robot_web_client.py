#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
)

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class RobotWebClient(Node):

    def __init__(self):
        super().__init__('robot_client_web')
        self.sub_gps_fix_ = self.create_subscription(
            NavSatFix, '/gps/fix', self.sub_gps_fix_cb, qos_profile_sensor_data
        )

        self.sub_odom_ = self.create_subscription(
            Odometry, '/odometry/filtered', self.sub_odom_cb, qos_profile_sensor_data
        )
        
        timer_period_ = 1.0
        # self.timer_ = self.create_timer(timer_period_, self.timer_callback)

        self.items = {
            'gps': {},
            'speed': {}
        }

    def sub_gps_fix_cb(self, msg):
        self.items['gps'] = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }

    def sub_odom_cb(self, msg):
        self.items['speed'] = {
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z
        }

    def getSpeed(self):
        return str(list(self.items['speed'].values()))

    def getGPS(self):
        return str(list(self.items['gps'].values()))

    def timer_callback(self):
        print(self.items)
        # print(self.getSpeed())
        # print(self.getGPS())


def main(args=None):
    rclpy.init(args=args)

    robot_client_web = RobotWebClient()

    rclpy.spin(robot_client_web)

    robot_client_web.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
