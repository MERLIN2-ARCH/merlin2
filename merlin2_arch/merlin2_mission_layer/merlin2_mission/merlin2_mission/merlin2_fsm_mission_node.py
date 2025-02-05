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


""" Base class to create MERLIN2 missions using a FSM """

from typing import List
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from .merlin2_mission_node import Merlin2MissionNode


class Merlin2FsmMissionNode(Merlin2MissionNode, StateMachine):
    """MERLIN2 FSM Mission Node Class"""

    def __init__(
        self,
        node_name: str,
        reset_problem: bool = True,
        run_mission: bool = False,
        outcomes: List[str] = None,
    ) -> None:

        if not outcomes:
            outcomes = [SUCCEED, ABORT, CANCEL]

        StateMachine.__init__(self, outcomes)
        Merlin2MissionNode.__init__(self, node_name, reset_problem, run_mission)

        set_ros_loggers(self)

        YasminViewerPub(node_name.upper(), self, node=self)

    def execute_mission(self) -> None:
        self()
