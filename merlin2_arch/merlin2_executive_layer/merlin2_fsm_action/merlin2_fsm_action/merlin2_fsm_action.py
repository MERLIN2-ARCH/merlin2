
""" MERLIN2 FSM Action """

from typing import Dict
from merlin2_arch_interfaces.msg import PlanAction
from merlin2_action.merlin2_action import Merlin2Action
from ros2_fsm_viewer import Ros2FsmViewerPub
from ros2_fsm.basic_fsm import StateMachine, State
from ros2_fsm.basic_fsm.shared_data import SharedData
from ros2_fsm.ros2_states import BasicOutomes, AcionState


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

        shared_data = SharedData()
        shared_data.merlin2_action_goal = goal

        outcome = self(shared_data)

        if outcome == BasicOutomes.SUCC:
            return True
        else:
            return False

    def cancel_action(self):
        self.cancel_state()

    def destroy(self):
        for action_state in self.__action_state_list:
            action_state.destroy()
        super().destroy()

    def add_state(self,
                  name: str,
                  state: State,
                  transitions: Dict[str, str] = None):

        if isinstance(state, AcionState):
            self.__action_state_list.append(state)

        super().add_state(name, state, transitions)
