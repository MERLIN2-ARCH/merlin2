
""" Custom action server that add goals to a queue """

import collections
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionQueueServer(ActionServer):
    """ Action Queue Goal Server Class """

    def __init__(self, node, s_type, s_name, execute_callback, cancel_callback=None):

        self.__user_execute_callback = execute_callback
        self.__user_cancel_callback = cancel_callback

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        super().__init__(node, s_type, s_name,
                         execute_callback=self.__execute_callback,
                         goal_callback=self.__goal_callback,
                         handle_accepted_callback=self.__handle_accepted_callback,
                         cancel_callback=cancel_callback,
                         callback_group=ReentrantCallbackGroup())

    def __handle_accepted_callback(self, goal_handle):
        """
            handle accepted calback for a queue goal server
            goals are added to a queue
        """

        with self._goal_queue_lock:
            if self._current_goal is not None:
                self._goal_queue.append(goal_handle)
            else:
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __goal_callback(self, goal_request):
        """ goal callback for a queue goal server """

        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle):
        """ cancel calback for a queue goal server """

        if self.__user_cancel_callback is not None:
            self.__user_cancel_callback()

        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle):
        """ 
            execute callback for a queue goal server
            when a goals ends, next goal is got from the queue
        """

        try:
            return self.__user_execute_callback(goal_handle)

        finally:
            with self._goal_queue_lock:
                try:
                    self._current_goal = self._goal_queue.popleft()
                    self._current_goal.execute()
                except IndexError:
                    self._current_goal = None
