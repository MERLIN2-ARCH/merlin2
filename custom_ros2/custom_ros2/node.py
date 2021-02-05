
""" ROS2 Node to simulate ROS1 Node """

from threading import Thread
from rclpy.node import Node as Node2
from rclpy.executors import MultiThreadedExecutor, Executor
from .actions import (
    ActionClient,
    ActionQueueServer,
    ActionSingleServer
)


class Node(Node2):
    """ Node Class """

    def __init__(self, node_name: str, namespace: str = "", executor: Executor = None):

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
            self._executor.wake()
            self._spin_thread.join()
        finally:
            self.get_logger().info("Destroying node " + self.get_name())
            self.destroy_node()

    def create_action_client(self, action_type, action_name: str) -> ActionClient:
        """ create action client from node

        Args:
            action_type ([type]): action type (msg action)
            action_name (str): action name

        Returns:
            ActionClient: client created
        """

        return ActionClient(self, action_type, action_name)

    def create_action_server(self,
                             action_type,
                             action_name,
                             execute_callback: str,
                             cancel_callback=None) -> ActionSingleServer:
        """ create action server from node

        Args:
            action_type ([type]): action type (msg action)
            action_name (str): action name
            execute_callback ([type]): execute function
            cancel_callback ([type], optional): cancel function. Defaults to None.

        Returns:
            ActionSingleServer: server created
        """

        return ActionSingleServer(self,
                                  action_type,
                                  action_name,
                                  execute_callback,
                                  cancel_callback=cancel_callback)

    def create_action_queue_server(self,
                                   action_type,
                                   action_name,
                                   execute_callback: str,
                                   cancel_callback=None) -> ActionQueueServer:
        """ create action queue server from node

        Args:
            action_type ([type]): action type (msg action)
            action_name (str): action name
            execute_callback ([type]): execute function
            cancel_callback ([type], optional): cancel function. Defaults to None.

        Returns:
            ActionQueueServer: queue server created
        """

        return ActionQueueServer(self,
                                 action_type,
                                 action_name,
                                 execute_callback,
                                 cancel_callback=cancel_callback)
