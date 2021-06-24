

from ros2_fsm.basic_fsm import State
from custom_ros2 import Node
from .basic_outcomes import BasicOutomes


class AcionState(State):

    def __init__(self,
                 node: Node,
                 action_type,
                 action_name: str,
                 create_goal_handler,
                 outcomes=None,
                 resutl_handler=None):

        _outcomes = [BasicOutomes.SUCC, BasicOutomes.ABOR, BasicOutomes.CANC]

        if outcomes:
            _outcomes = _outcomes + outcomes

        self.__action_client = node.create_action_client(
            action_type, action_name)

        self.__create_goal_handler = create_goal_handler
        self.__resutl_handler = resutl_handler

        if not self.__create_goal_handler or not self.__resutl_handler:
            raise Exception("create_goal and result handlers are needed")

        super().__init__(_outcomes)

    def _create_goal(self, blackboard):
        return self.__create_goal_handler(blackboard)

    def cancel_state(self):
        self.__action_client.cancel_goal()
        super().cancel_state()

    def execute(self, blackboard):

        goal = self._create_goal(blackboard)
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_canceled():
            return BasicOutomes.CANC
        elif self.__action_client.is_aborted():
            return BasicOutomes.ABOR
        elif self.__action_client.is_succeeded():

            result = self.__action_client.get_result()
            outcome = self.__resutl_handler(blackboard, result)

            if outcome:
                return outcome

            return BasicOutomes.SUCC
