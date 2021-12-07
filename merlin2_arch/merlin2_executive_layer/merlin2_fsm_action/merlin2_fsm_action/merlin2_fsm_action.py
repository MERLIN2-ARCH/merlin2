
""" MERLIN2 FSM Action """

from merlin2_arch_interfaces.msg import PlanAction
from merlin2_action.merlin2_action import Merlin2Action
from yasmin_viewer import YasminViewerPub
from yasmin import StateMachine, State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from .merlin2_state_factory import Merlin2StateFactory


class Merlin2FsmAction(Merlin2Action, StateMachine):
    """ Merlin2 FSM Action Class """

    def __init__(self, action_name: str):

        self.__state_factory = Merlin2StateFactory()

        Merlin2Action.__init__(self, action_name)
        StateMachine.__init__(self, [SUCCEED, ABORT, CANCEL])

        YasminViewerPub(self, action_name.upper(), self)

    def __hash__(self):
        return Merlin2Action.__hash__(self)

    def run_action(self, goal: PlanAction) -> bool:

        blackboard = Blackboard()
        blackboard.merlin2_action_goal = goal

        outcome = self(blackboard)

        return outcome == SUCCEED

    def cancel_action(self):
        self.cancel_state()

    def create_state(self, state: int) -> State:
        """ create the basic state

        Args:
            state (int): state from basic states

        Returns:
            State: state object
        """

        return self.__state_factory.create_state(state, node=self)
