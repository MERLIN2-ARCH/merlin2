
""" Merlin2 Action Base """

from merlin2_plan_sys_interfaces.action import DispatchAction

from custom_ros2 import (
    Node,
    ActionSingleServer
)


class Merlin2Action(Node):
    """ Merlin2 Action Class """

    def __init__(self, a_name):

        super().__init__(a_name)

        self._is_canceled = False

        self.__action_server = ActionSingleServer(self, DispatchAction,
                                                  "merlin2_action/" + a_name,
                                                  self.__execute_server,
                                                  cancel_callback=self.__cancel_callback)

    def destroy(self):
        """ destroy node method """

        self.__action_server.destroy()
        super().destroy_node()

    def run_action(self) -> bool:
        """ Code of the action. Must be implemented.

        Returns:
            bool: action succeed
        """

        return NotImplementedError()

    def get_is_canceled(self) -> bool:
        """ get if action is canceled

        Returns:
            bool: action is canceled?
        """

        return self._is_canceled

    def __cancel_callback(self):
        self._is_canceled = True

    def __execute_server(self, goal_handle):
        self._is_canceled = True
        result = DispatchAction.Result()

        succeed = self.run_action()

        if self._is_canceled:
            goal_handle.canceled()

        else:
            if succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        return result
