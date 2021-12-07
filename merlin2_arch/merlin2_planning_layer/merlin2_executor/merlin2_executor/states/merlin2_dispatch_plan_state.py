""" Dispatch Plan State """

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import AcionState
from yasmin_ros.basic_outcomes import SUCCEED
from merlin2_arch_interfaces.action import DispatchPlan


class Merlin2DispatchPlanState(AcionState):
    def __init__(self, node: Node):
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
