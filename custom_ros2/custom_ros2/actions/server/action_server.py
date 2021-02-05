
""" Custom action server that add goals to a queue """


import time
from rclpy.action import ActionServer as ActionServer2
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class ActionServer(ActionServer2):
    """ Action Server Class """

    def __init__(self,
                 node,
                 action_type,
                 action_name,
                 execute_callback,
                 handle_accepted_callback,
                 cancel_callback=None):

        self.__user_execute_callback = execute_callback
        self.__user_handle_accepted_callback = handle_accepted_callback
        self.__user_cancel_callback = cancel_callback
        self.__server_canceled = False
        self._goal_handle = None

        super().__init__(node, action_type, action_name,
                         execute_callback=self.__execute_callback,
                         goal_callback=self.__goal_callback,
                         handle_accepted_callback=self.__handle_accepted_callback,
                         cancel_callback=self.__cancel_callback,
                         callback_group=ReentrantCallbackGroup())

    def is_canceled(self) -> bool:
        """ get if server is canceled

        Returns:
            bool: server canceled?
        """

        return self.__server_canceled

    def wait_for_canceling(self):
        """
            if server is canceled, wait for canceling state
        """

        if self.__server_canceled and self._goal_handle:
            while not self._goal_handle.is_cancel_requested:
                time.sleep(0.05)

    def __goal_callback(self, goal_request):
        """ goal callback """

        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle):
        """ cancel calback """

        self.__server_canceled = True

        if self.__user_cancel_callback is not None:
            self.__user_cancel_callback()

        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle):
        """
            execute callback
        """

        self._goal_handle = goal_handle
        self.__server_canceled = False
        results = self.__user_execute_callback(goal_handle)
        self._goal_handle = None
        return results

    def __handle_accepted_callback(self, goal_handle):
        if self.__user_handle_accepted_callback:
            self.__user_handle_accepted_callback(goal_handle)
        goal_handle.execute()
