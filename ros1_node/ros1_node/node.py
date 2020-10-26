
""" ROS2 Node to simulate ROS1 Node """

from threading import Thread
import rclpy
from rclpy.node import Node as Node2
from rclpy.executors import MultiThreadedExecutor


class Node(Node2):
    """ Node Class """

    def __init__(self, node_name):
        super().__init__(node_name)
        self._threading_spin()

    def _run_executor(self):
        self._executor.add_node(self)
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()

    def _threading_spin(self):
        self._executor = MultiThreadedExecutor()
        self._spin_thread = Thread(target=self._run_executor, daemon=True)
        self._spin_thread.start()

    def wait_spinning(self):
        self._spin_thread.join()
