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


""" Generate Plan State """

import time
from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from merlin2_msgs.srv import GeneratePlan


class Merlin2GeneratePlanState(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            GeneratePlan, "generate_plan",
            self.create_request_handler,
            response_handler=self.response_handler,
            node=node
        )
        self._node = node

    def create_request_handler(self, blackboard: Blackboard) -> GeneratePlan.Request:
        req = GeneratePlan.Request()
        req.domain = blackboard["domain"]
        req.problem = blackboard["problem"]
        return req

    def response_handler(self, blackboard: Blackboard, response: GeneratePlan.Response) -> str:

        self._node.get_logger().info("Plan has solution: " + str(response.has_solution))
        if not response.has_solution:
            return ABORT

        self._node.get_logger().info(str(response.plan))
        blackboard["plan"] = response.plan
        blackboard["result"].generate_plan = True

        elapsed_time = time.time() - blackboard["init_time"]
        self._node.get_logger().info(
            f"Time from receiving goals to planning: {elapsed_time} seconds")

        return SUCCEED
