
""" Base class to create MERLIN2 missions using a FSM """

from typing import List
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from .merlin2_fsm_mission_node import Merlin2MissionNode


class Merlin2FsmMissionNode(Merlin2MissionNode, StateMachine):
    """ MERLIN2 FSM Mission Node Class """

    def __init__(self,
                 node_name: str,
                 reset_problem: bool = True,
                 run_mission: bool = False,
                 outcomes: List[str] = None):

        if not outcomes:
            outcomes = [SUCCEED, ABORT, CANCEL]

        StateMachine.__init__(self, outcomes)
        Merlin2MissionNode.__init__(
            self, node_name, reset_problem, run_mission)

        YasminViewerPub(self, node_name.upper(), self)

    def execute_mission(self):
        self()
