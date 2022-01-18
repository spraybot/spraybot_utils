#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
)

from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsDatum(Node):
    """
    Republishes the first valid gps fix and sets datum in robot_localization.

    Subscribes and stores the first valid gps fix, then republishes it as the
    origin. Also calls SetDatum service offered by robot_localization.

    """

    def __init__(self):
        super().__init__('gps_datum')

        self.gps_datm_msg_ = None
        self.rl_datum_future_ = None
        self.rl_datum_set_ = False

        self.sub_gps_ = self.create_subscription(
            NavSatFix, 'gps/fix', self.sub_gps_cb, qos_profile_sensor_data
        )

        self.pub_gps_datum_ = self.create_publisher(
            NavSatFix,
            'gps/fix/origin',
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # Need to use a timer since latching behaviour doesn't behave like ROS1
        # https://github.com/ros2/ros2/issues/464
        timer_period_ = 1.0
        self.timer_ = self.create_timer(timer_period_, self.timer_callback)

        self.rl_datum_client = self.create_client(SetDatum, 'datum')
        self.get_logger().info('Waiting for robot_localization datum service')
        self.rl_datum_client.wait_for_service()
        self.get_logger().info('robot_localization datum service now available')

    def sub_gps_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self.gps_datm_msg_ = msg
        self.get_logger().info('Successfully set origin. Unsubscribing to gps fix')
        self.destroy_subscription(self.sub_gps_)

    def timer_callback(self):
        if self.gps_datm_msg_ is None:
            return
        self.pub_gps_datum_.publish(self.gps_datm_msg_)
        self.send_rl_request()

    def send_rl_request(self):
        if self.rl_datum_set_ or self.gps_datm_msg_ is None:
            return

        if self.rl_datum_future_ is None:
            req = SetDatum.Request()
            req.geo_pose.position.latitude = self.gps_datm_msg_.latitude
            req.geo_pose.position.longitude = self.gps_datm_msg_.longitude
            req.geo_pose.position.altitude = self.gps_datm_msg_.altitude
            req.geo_pose.orientation.w = 1.0
            self.get_logger().info(
                'Sending request to SetDatum request to robot_localization'
            )
            self.rl_datum_future_ = self.rl_datum_client.call_async(req)
        else:
            if self.rl_datum_future_.done():
                try:
                    self.rl_datum_future_.result()
                # Flake8 throws blind exception error (https://github.com/ros2/examples/issues/325)
                except Exception as e:  # noqa: B902
                    self.get_logger().info(
                        'Call to SetDatum service in robot_localization failed %r'
                        % (e,)
                    )
                else:
                    self.get_logger().info('Datum set in robot_localization')
                    self.rl_datum_set_ = True


def main(args=None):
    rclpy.init(args=args)

    gps_datum = GpsDatum()

    rclpy.spin(gps_datum)

    gps_datum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
