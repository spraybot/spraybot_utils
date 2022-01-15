#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vectornav_msgs.msg import AttitudeGroup, CommonGroup, GpsGroup, InsGroup


class Item:

    def __init__(self, label, value_format, default_value):
        self.label = label
        self.value_format = value_format
        if not isinstance(default_value, list):
            default_value = [default_value]
        self.value = default_value

        self.colors = {'white': '#ffffff', 'red': '#ff0000', 'green': '#00ff00'}

        self.label_color = self.value_color = self.colors['white']

    def html(self):
        html_str = "<p style='color: {lbl_color};font-weight: bold'>{lbl}: ".format(
            lbl_color=self.label_color, lbl=self.label
        )
        html_str += (
            "<span style='color: {val_color}'>" + self.value_format + '</span><p>'
        )
        return html_str.format(*self.value, val_color=self.value_color)

    def update_value(self, val, color='white'):
        if not isinstance(val, list):
            val = [val]
        self.value = val
        self.update_value_format(color)

    def update_value_format(self, color='white'):
        # TODO: Add check if color is valid
        self.value_color = self.colors[color]


class VectornavHtmlStatus(Node):

    def __init__(self):
        super().__init__('vectornav_html_status')
        self.sub_vn_common_ = self.create_subscription(
            CommonGroup, 'vectornav/raw/common', self.sub_vn_common_cb, 11
        )

        self.sub_vn_attitude_ = self.create_subscription(
            AttitudeGroup, 'vectornav/raw/attitude', self.sub_vn_attitude_cb, 1
        )

        self.sub_vn_common_ = self.create_subscription(
            GpsGroup, 'vectornav/raw/gps', self.sub_vn_gps_cb, 1
        )

        self.sub_vn_common_ = self.create_subscription(
            InsGroup, 'vectornav/raw/ins', self.sub_vn_ins_cb, 1
        )

        self.html_publisher_ = self.create_publisher(
            String, 'vectornav/html_status', 10
        )
        timer_period_ = 0.5
        self.timer_ = self.create_timer(timer_period_, self.timer_callback)

        self.items = {
            'fix': Item('Fix', '{}', ''),
            'gpsu': Item('GPSU', '{:.2f}, {:.2f}, {:.2f}', [0.0, 0.0, 0.0]),
            'insu': Item('INSU', '{:.2f}', 0.0),
            'yaw': Item('Yaw', '{:+.2f}', 0.0),
        }

    def sub_vn_common_cb(self, msg):
        self.items['yaw'].update_value(msg.yawpitchroll.x)

    def sub_vn_attitude_cb(self, msg):
        pass

    def sub_vn_gps_cb(self, msg):
        fix = ''
        fix_color = 'green'
        if msg.fix == 2:
            fix = '2D'
        elif msg.fix == 3:
            fix = '3D'
        elif msg.fix == 4:
            fix = 'SBAS'
        else:
            fix = 'NO_FIX'
            fix_color = 'red'
        self.items['fix'].update_value(fix, color=fix_color)

        self.items['gpsu'].update_value([msg.posu.x, msg.posu.y, msg.posu.z])

    def sub_vn_ins_cb(self, msg):
        self.items['insu'].update_value(msg.posu)

    def timer_callback(self):
        msg = String()
        msg.data = self.generate_html_string()
        self.html_publisher_.publish(msg)

    def generate_html_string(self):
        html_str = ''
        for _, value in self.items.items():
            html_str += value.html()
        return html_str


def main(args=None):
    rclpy.init(args=args)

    vectornav_html_status = VectornavHtmlStatus()

    rclpy.spin(vectornav_html_status)

    vectornav_html_status.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
