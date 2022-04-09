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
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool
import numpy as np


class RobotWebClient(Node):

    def __init__(self):
        super().__init__('robot_client_web')
        self.sub_gps_fix_ = self.create_subscription(
            NavSatFix, '/gps/fix', self.sub_gps_fix_cb, qos_profile_sensor_data
        )

        self.sub_odom_ = self.create_subscription(
            Odometry, '/odometry/filtered', self.sub_odom_cb, qos_profile_sensor_data
        )

        self.sub_diagnostics_ = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.sub_diag_cb, qos_profile_sensor_data
        )

        self.sub_autonomous_mode_ = self.create_subscription(
            Joy, '/joy_teleop/joy', self.sub_auto_mode_cb, qos_profile_sensor_data
        )

        self.sub_in_row_status_ = self.create_subscription(
            Bool, '/in_row_flag', self.sub_in_row_status_cb, qos_profile_sensor_data
        )
        
        timer_period_ = 1.0
        self.timer_ = self.create_timer(timer_period_, self.timer_callback)

        self.items = {
            'gps': {'latitude': 0,
            'longitude': 0,
            'latitude': 0},
            'speed': {'linear': 0,
            'angular': 0},
            'battery': 0,
            'in_row_status': False,
            'autonomous_mode': True
        }

    def sub_gps_fix_cb(self, msg):
        self.items['gps'] = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'latitude': msg.altitude
        }

    def sub_odom_cb(self, msg):
        self.items['speed'] = {
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z
        }
    
    def sub_diag_cb(self, msg):
        for vals in msg.status[0].values:
            if (vals.key == 'Battery Voltage'):
                batt_percentage = ( (float(vals.value) - 20) / (28.8 - 20) ) * (100 - 0) + 6
                self.items['battery'] = batt_percentage

    def sub_in_row_status_cb(self,msg):
        self.items['in_row_status'] = msg.data

    def sub_auto_mode_cb(self,msg):
        if ((msg.buttons[4] == 1) or (msg.buttons[5] == 1)):
            self.items['autonomous_mode'] = False
        else:
            self.items['autonomous_mode'] = True


    def getSpeed(self):
        return str(list(self.items['speed'].values()))

    def getGPS(self):
        return str(list(self.items['gps'].values()))

    def timer_callback(self):
        print(self.items)
 
def main(args=None):
    rclpy.init(args=args)

    robot_client_web = RobotWebClient()

    rclpy.spin(robot_client_web)

    robot_client_web.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
