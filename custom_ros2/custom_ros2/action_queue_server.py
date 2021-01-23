
""" Custom action server that add goals to a queue """

import collections
import threading
from .action_server import ActionServer


class ActionQueueServer(ActionServer):
    """ Action Queue Goal Server Class """

    def __init__(self, node, action_type, action_name, execute_callback, cancel_callback=None):

        self.__queue_user_execute_callback = execute_callback

        self.__goal_queue = collections.deque()
        self.__goal_queue_lock = threading.Lock()

        super().__init__(node, action_type, action_name,
                         execute_callback=self.__queue_execute_callback,
                         handle_accepted_callback=self.__queue_handle_accepted_callback,
                         cancel_callback=cancel_callback)

    def __queue_handle_accepted_callback(self, goal_handle):
        """
            handle accepted calback for a queue goal server
            goals are added to a queue
        """

        with self.__goal_queue_lock:
            if self._goal_handle is not None:
                self.__goal_queue.append(goal_handle)
            else:
                self._goal_handle = goal_handle
                self._goal_handle.execute()

    def __queue_execute_callback(self, goal_handle):
        """
            execute callback for a queue goal server
            when a goals ends, next goal is got from the queue
        """

        try:
            return self.__queue_user_execute_callback(goal_handle)

        finally:
            with self.__goal_queue_lock:
                try:
                    self._goal_handle = self.__goal_queue.popleft()
                    self._goal_handle.execute()
                except IndexError:
                    self._goal_handle = None
