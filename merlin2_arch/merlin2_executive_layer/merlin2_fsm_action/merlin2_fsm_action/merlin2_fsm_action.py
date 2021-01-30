
""" MERLIN2 FSM Action """

from merlin2_arch_interfaces.msg import PlanAction
from merlin2_action.merlin2_action import Merlin2Action
from ros2_fsm_viewer import Ros2FsmViewerPub
from ros2_fsm.basic_fsm import StateMachine
from ros2_fsm.basic_fsm.blackboard import Blackboard
from ros2_fsm.ros2_states import BasicOutomes


class Merlin2FsmAction(Merlin2Action, StateMachine):
    """ Merlin2 FSM Action Class """

    def __init__(self, action_name: str):

        self.__action_state_list = []

        Merlin2Action.__init__(self, action_name)
        StateMachine.__init__(self, [BasicOutomes.SUCC,
                                     BasicOutomes.ABOR,
                                     BasicOutomes.CANC])

        Ros2FsmViewerPub(self, action_name.upper(), self)

    def __hash__(self):
        return Merlin2Action.__hash__(self)

    def run_action(self, goal: PlanAction) -> bool:

        blackboard = Blackboard()
        blackboard.merlin2_action_goal = goal

        outcome = self(blackboard)

        if outcome == BasicOutomes.SUCC:
            return True
        else:
            return False

    def cancel_action(self):
        self.cancel_state()
