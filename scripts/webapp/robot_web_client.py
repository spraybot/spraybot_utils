#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    # api_req_qos_profile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
)

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature
from husky_msgs.msg import HuskyStatus
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

        self.sub_temp_pc_ = self.create_subscription(
            HuskyStatus, '/status', self.sub_temp_pc_cb, qos_profile_sensor_data
        )

        self.sub_temp_enclosure_ = self.create_subscription(
            Temperature, '/temp_data_enclosure', self.sub_temp_enclosure_cb, qos_profile_sensor_data
        )
        
        self.auto_start_flag = True 
        # TODO: verify topic and qos profile
        self.velocity_lock_pub = self.create_publisher(
            Bool, '/webapp/e_stop', 10
        )

        timer_period_ = 0.25
        self.timer_ = self.create_timer(timer_period_, self.timer_callback)

        self.items = {
            'gps': {'latitude': 0,
            'longitude': 0,
            'latitude': 0},
            'speed': {'linear': 0,
            'angular': 0},
            'battery': 0,
            'in_row_status': False,
            'autonomous_mode': True,
            'temperature_pc': 24,
            'temperature_enclosure': 24
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
    
    def sub_diag_cb(self, msg):
        for vals in msg.status[0].values:
            if (vals.key == 'Battery Voltage'):
                batt_percentage = ( (float(vals.value) - 20) / (28.8 - 20) ) * (20 - 0) + 6
                self.items['battery'] = batt_percentage

    def sub_in_row_status_cb(self,msg):
        self.items['in_row_status'] = msg.data

    def sub_auto_mode_cb(self,msg):
        if ((msg.buttons[4] == 1) or (msg.buttons[5] == 1)):
            self.items['autonomous_mode'] = False
        else:
            self.items['autonomous_mode'] = True

    def sub_temp_pc_cb(self, msg):
        self.items['temperature_pc'] = msg.left_driver_current + msg.right_driver_current

    def sub_temp_enclosure_cb(self, msg):
        self.items['temperature_enclosure'] = msg.temperature

    def getSpeed(self):
        return str(list(self.items['speed'].values()))

    def getGPSLatitude(self):
        return str(list(self.items['gps'].values())[0])

    def getGPSLongtitude(self):
        return str(list(self.items['gps'].values())[1])

    def getBattery(self):
        return str(self.items['battery'])

    def getInRowStat(self):
        return str(self.items['in_row_status'])

    def getAutoMode(self):
        return self.items['autonomous_mode']

    def getTempPC(self):
        return str(self.items['temperature_pc'])

    def getTempEnclosure(self):
        return str(self.items['temperature_enclosure'])

    def setStartAutoFlag(self, flag):
        self.auto_start_flag = flag 

    def timer_callback(self):
        msg = Bool()
        msg.data = not self.auto_start_flag
        self.velocity_lock_pub.publish(msg)
        # print(self.getGPSLatitude())
        # print(self.getGPSLongtitude())
 
def main(args=None):
    rclpy.init(args=args)

    robot_client_web = RobotWebClient()

    rclpy.spin(robot_client_web)

    robot_client_web.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
