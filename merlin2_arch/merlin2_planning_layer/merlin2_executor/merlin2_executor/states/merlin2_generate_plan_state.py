""" Generate Plan State """

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from merlin2_arch_interfaces.srv import GeneratePlan


class Merlin2GeneratePlanState(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(node, GeneratePlan, "generate_plan",
                         self.create_request_handler, response_handler=self.response_handler)
        self._node = node

    def create_request_handler(self, blackboard: Blackboard) -> GeneratePlan.Request:
        req = GeneratePlan.Request()
        req.domain = blackboard.domain
        req.problem = blackboard.problem
        return req

    def response_handler(self, blackboard: Blackboard, response: GeneratePlan.Response) -> str:

        self._node.get_logger().info("Plan has solution: " + str(response.has_solution))
        if not response.has_solution:
            return ABORT

        self._node.get_logger().info(str(response.plan))
        blackboard.plan = response.plan
        blackboard.result.generate_plan = True

        return SUCCEED
