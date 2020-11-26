
""" Merlin2 Plan Dispatcher Node """

import threading
import collections
from merlin2_plan_sys_interfaces.action import (
    DispatchPlan,
    DispatchAction
)
import rclpy
from rclpy.action import ActionServer, CancelResponse

from threaded_node.node import Node


class Merlin2PlanDispatcherNode(Node):
    """ Merlin2 Plan Dispatcher Node Class """

    def __init__(self):

        super().__init__("merlin2_plan_dispatcher_mode")

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        # action server
        self._action_server = ActionServer(self,
                                           DispatchPlan,
                                           "dispatch_plan",
                                           execute_callback=self.__execute_server,
                                           cancel_callback=self.__cancel_server,
                                           handle_accepted_callback=self.__accepted_callback,
                                           )

    def destroy(self):
        """ destroy node method
        """

        self._action_server.destroy()
        super().destroy_node()

    def __accepted_callback(self, goal_handle):
        """ action server accepted callback or defer execution of an already accepted goal

        Args:
            goal_handle: goal handle
        """

        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info("Goal put in the queue")
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __cancel_server(self, goal_handle):
        """ action server cancel callback

        Args:
            goal_handle: goal handle

        Returns:
            CancelResponse: cancel response
        """

        self.get_logger().info("Cancelling Plan Dispatcher")
        return CancelResponse.ACCEPT

    def __execute_server(self, goal_handle):
        """action server execute callback"""

        try:

            result = DispatchPlan.Result()

            for action in goal_handle.request.plan:
                self.get_logger().info("Executin action " + str(action.action_name) +
                                       " with objects " + str(action.objects))

            goal_handle.succeed()

            return result

        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info("Next goal pulled from the queue")
                    self._current_goal.execute()

                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PlanDispatcherNode()

    node.join_spin()

    node.destroy()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
