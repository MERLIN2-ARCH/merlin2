

from ros2_fsm.basic_fsm import State
from custom_ros2 import ActionClient
from .basic_outcomes import BasicOutomes


class AcionState(State):

    def __init__(self, node, action_type, action_name, create_goal_handler, resutl_handler=None):

        self.__action_client = ActionClient(node, action_type, action_name)

        self.__create_goal_handler = create_goal_handler
        self.__resutl_handler = resutl_handler

        super().__init__([BasicOutomes.SUCC,
                          BasicOutomes.ABOR,
                          BasicOutomes.CANC])

    def _create_goal(self, shared_data):
        return self.__create_goal_handler(shared_data)

    def cancel_state(self):
        super().cancel_state()
        self.__action_client.cancel_goal()

    def execute(self, shared_data):

        goal = self._create_goal(shared_data)
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_canceled():
            return BasicOutomes.CANC
        elif self.__action_client.is_aborted():
            return BasicOutomes.ABOR
        elif self.__action_client.is_succeeded():

            if self.__resutl_handler:
                result = self.__action_client.get_result()
                self.__resutl_handler(shared_data, result)

            return BasicOutomes.SUCC
