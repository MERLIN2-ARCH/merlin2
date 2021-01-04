
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse


class ActionSingleServer(ActionServer):

    def __init__(self, node, s_type, s_name, execute_callback, cancel_callback=None):

        if cancel_callback is None:
            cancel_callback = self.__cancel_callback

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        super().__init__(node, s_type, s_name,
                         execute_callback=execute_callback,
                         goal_callback=self.__goal_callback,
                         handle_accepted_callback=self.__handle_accepted_callback,
                         cancel_callback=cancel_callback)

    def __goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def __handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def __cancel_callback(self, goal):
        return CancelResponse.ACCEPT
