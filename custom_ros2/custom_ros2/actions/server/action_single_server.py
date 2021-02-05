
""" Custom action server that treats only one goals at the same time """

import threading
from .action_server import ActionServer


class ActionSingleServer(ActionServer):
    """ Action Single Goal Server Class """

    def __init__(self, node, action_type, action_name, execute_callback, cancel_callback=None):

        self.__goal_lock = threading.Lock()

        super().__init__(node, action_type, action_name,
                         execute_callback=execute_callback,
                         handle_accepted_callback=self.__single_handle_accepted_callback,
                         cancel_callback=cancel_callback)

    def __single_handle_accepted_callback(self, goal_handle):
        """
            handle accepted calback for a single goal server
            only one goal can be treated
            if other goal is send, old goal is aborted and replaced with the new one
        """

        with self.__goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()

            self._goal_handle = goal_handle
