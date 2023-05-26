""" Generate PDDL State """

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED
from merlin2_msgs.srv import GeneratePddl


class Merlin2GeneratePddlState(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(node, GeneratePddl, "generate_pddl",
                         self.create_request_handler, response_handler=self.response_handler)
        self._node = node

    def create_request_handler(self, blackboard: Blackboard) -> GeneratePddl.Request:
        return GeneratePddl.Request()

    def response_handler(self, blackboard: Blackboard, response: GeneratePddl.Response) -> str:
        blackboard.domain = response.domain
        blackboard.problem = response.problem

        self._node.get_logger().info(response.domain)
        self._node.get_logger().info(response.problem)

        blackboard.result.generate_pddl = True

        return SUCCEED
