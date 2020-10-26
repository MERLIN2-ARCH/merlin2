
""" ROS2 Node to simulate ROS1 Node """

from threading import Thread
from rclpy.node import Node as Node2
from rclpy.executors import MultiThreadedExecutor, Executor


class Node(Node2):
    """ Node Class """

    def __init__(self, node_name, executor: Executor = None):
        super().__init__(node_name)

        if not executor:
            self._executor = MultiThreadedExecutor()

        self._spin_thread = Thread(target=self._run_executor, daemon=True)
        self._spin_thread.start()

    def _run_executor(self):
        self._executor.add_node(self)
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()

    def join_spin(self):
        """ wait for spin thread """

        self._spin_thread.join()
