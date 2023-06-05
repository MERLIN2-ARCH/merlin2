# Copyright (C) 2023  Miguel Ángel González Santamarta

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


""" Dispatch Plan State """

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import AcionState
from yasmin_ros.basic_outcomes import SUCCEED
from merlin2_msgs.action import DispatchPlan


class Merlin2DispatchPlanState(AcionState):
    def __init__(self, node: Node) -> None:
        super().__init__(node, DispatchPlan, "dispatch_plan",
                         self.create_goal_handler, result_handler=self.result_handler)
        self._node = node

    def create_goal_handler(self, blackboard: Blackboard) -> DispatchPlan.Goal:
        goal = DispatchPlan.Goal()
        goal.plan = blackboard.plan
        return goal

    def result_handler(self, blackboard: Blackboard, response: DispatchPlan.Result) -> str:
        blackboard.result.dispatch_plan = True
        return SUCCEED
