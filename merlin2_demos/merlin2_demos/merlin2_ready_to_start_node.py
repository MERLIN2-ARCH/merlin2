#!/usr/bin/env python3

import rclpy
from simple_node import Node
from std_msgs.msg import String


class Merlin2ReadyToStart(Node):

    def __init__(self):

        super().__init__("merlin2_ready_to_start")
        self.publisher_ = self.create_publisher(String, "keyboard_input", 10)
        self.timer_ = self.create_timer(1.0, self.publish_keyboard_input)
    
    def publish_keyboard_input(self):
        entrada = input("Key 'Enter' to start: ")
        msg = String()
        msg.data = entrada
        self.get_logger().info(f'{entrada}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node_ready_to_start = Merlin2ReadyToStart()
    node_ready_to_start.join_spin()

    node_ready_to_start.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
