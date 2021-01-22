
""" Custom action server that treats only one goals at the same time """

import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionSingleServer(ActionServer):
    """ Action Single Goal Server Class """

    def __init__(self, node, s_type, s_name, execute_callback, cancel_callback=None):

        self.__user_cancel_callback = cancel_callback

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        super().__init__(node, s_type, s_name,
                         execute_callback=execute_callback,
                         goal_callback=self.__goal_callback,
                         handle_accepted_callback=self.__handle_accepted_callback,
                         cancel_callback=self.__cancel_callback,
                         callback_group=ReentrantCallbackGroup())

    def __goal_callback(self, goal_request):
        """ goal callback for a single goal server """

        return GoalResponse.ACCEPT

    def __handle_accepted_callback(self, goal_handle):
        """
            handle accepted calback for a single goal server
            only one goal can be treated
            if other goal is send, old goal is aborted and replaced with the new one
        """

        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def __cancel_callback(self, goal):
        """ cancel calback for a single goal server """

        if self.__user_cancel_callback is not None:
            self.__user_cancel_callback()

        self._goal_handle = None

        return CancelResponse.ACCEPT
