
""" ROS2 Node to simulate ROS1 Node """

from threading import Thread
from rclpy.node import Node as Node2
from rclpy.executors import MultiThreadedExecutor, Executor


class Node(Node2):
    """ Node Class """

    def __init__(self, node_name, namespace: str = "", executor: Executor = None):

        super().__init__(node_name, namespace=namespace)

        if not executor:
            self._executor = MultiThreadedExecutor()
        else:
            self._executor = executor

        self._spin_thread = Thread(target=self._run_executor, daemon=True)
        self._spin_thread.start()

    def _run_executor(self):
        """ run an executer with self (node) """

        self._executor.add_node(self)
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()

    def join_spin(self):
        """ wait for spin thread """

        try:
            self._spin_thread.join()
        finally:
            self.get_logger().info("Destroying node " + self.get_name())
            self.destroy_node()
