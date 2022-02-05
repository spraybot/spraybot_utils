#!/usr/bin/env python3
import threading

from getkey import getkey, keys

import rclpy
from rclpy.node import Node

from rosbag2_interfaces.srv import (
    IsPaused,
    Pause,
    Resume,
    TogglePaused
)


class PlayController(Node):

    def __init__(self):
        super().__init__('playcontrol')
        self.pause_client = self.create_client(Pause, '/rosbag2_player/pause')
        self.resume_client = self.create_client(Resume, '/rosbag2_player/resume')
        self.toggle_client = self.create_client(
            TogglePaused, '/rosbag2_player/toggle_paused'
        )
        self.is_paused_client = self.create_client(
            IsPaused, '/rosbag2_player/is_paused'
        )
        self.timer = self.create_timer(1, self.state_timer)

    def print_state(self, future):
        res = future.result()
        if res is not None:
            self.get_logger().info(f'IsPaused: {res.paused}')
        else:
            self.get_logger().error(
                f'Exception while calling service: {future.exception()}'
            )

    def trigger_result_cb(self, name):
        def impl(future):
            res = future.result()
            if res is not None:
                self.get_logger().info(f'Response received for: {name}')
            else:
                self.get_logger().error(f'Exception for service: {future.exception()}')

        return impl

    def state_timer(self):
        req = IsPaused.Request()
        future = self.is_paused_client.call_async(req)
        future.add_done_callback(self.print_state)

    def pause(self):
        req = Pause.Request()
        future = self.pause_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Pause'))

    def resume(self):
        req = Resume.Request()
        future = self.resume_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Resume'))

    def toggle_paused(self):
        req = TogglePaused.Request()
        future = self.toggle_client.call_async(req)
        future.add_done_callback(self.trigger_result_cb('Toggle'))


def spin_fn(node):
    node.get_logger().info('Starting spin thread')
    rclpy.spin(node)
    node.get_logger().info('Spin thread done')


def main():
    rclpy.init()
    node = PlayController()
    spin_thread = threading.Thread(target=spin_fn, args=[node])
    spin_thread.start()

    try:
        while True:
            c = getkey()
            if c == keys.SPACE:
                node.toggle_paused()
            elif c == keys.P:
                node.pause()
            elif c == keys.R:
                node.resume()
            else:
                print(f'Ignored: {c}')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
