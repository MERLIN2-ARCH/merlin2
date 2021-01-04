
import collections
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse


class ActionQueueServer(ActionServer):

    def __init__(self, node, s_type, s_name, execute_callback, cancel_callback=None):

        self.__user_execute_callback = execute_callback

        if cancel_callback is None:
            cancel_callback = self.__cancel_callback

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        super().__init__(node, s_type, s_name,
                         execute_callback=self.__execute_callback,
                         goal_callback=self.__goal_callback,
                         handle_accepted_callback=self.__handle_accepted_callback,
                         cancel_callback=cancel_callback)

    def __handle_accepted_callback(self, goal_handle):
        with self._goal_queue_lock:
            if self._current_goal is not None:
                self._goal_queue.append(goal_handle)
            else:
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle):
        try:
            return self.__user_execute_callback(goal_handle)

        finally:
            with self._goal_queue_lock:
                try:
                    self._current_goal = self._goal_queue.popleft()
                    self._current_goal.execute()
                except IndexError:
                    self._current_goal = None
