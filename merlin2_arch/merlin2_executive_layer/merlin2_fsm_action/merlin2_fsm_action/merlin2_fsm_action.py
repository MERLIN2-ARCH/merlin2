# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""MERLIN2 FSM Action"""

from merlin2_msgs.msg import PlanAction
from merlin2_action.merlin2_action import Merlin2Action

from yasmin import StateMachine, State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from merlin2_fsm_action.merlin2_state_factory import Merlin2StateFactory


class Merlin2FsmAction(Merlin2Action, StateMachine):
    """Merlin2 FSM Action Class"""

    def __init__(self, action_name: str) -> None:

        self.__state_factory = Merlin2StateFactory()

        Merlin2Action.__init__(self, action_name)
        StateMachine.__init__(self, [SUCCEED, ABORT, CANCEL])

        set_ros_loggers(self)

        YasminViewerPub(self.get_name().upper(), self, node=self)

    def __hash__(self):
        return Merlin2Action.__hash__(self)

    def run_action(self, goal: PlanAction) -> bool:

        blackboard = Blackboard()
        blackboard["merlin2_action_goal"] = goal

        outcome = self(blackboard)

        return outcome == SUCCEED

    def cancel_action(self) -> None:
        self.cancel_state()

    def create_state(self, state: int) -> State:
        """create the basic state

        Args:
            state (int): state from basic states

        Returns:
            State: state object
        """

        return self.__state_factory.create_state(state, node=self)
