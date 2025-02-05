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


"""Generate PDDL State"""

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED
from merlin2_msgs.srv import GeneratePddl


class Merlin2GeneratePddlState(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            GeneratePddl,
            "generate_pddl",
            self.create_request_handler,
            response_handler=self.response_handler,
            node=node,
        )
        self._node = node

    def create_request_handler(self, blackboard: Blackboard) -> GeneratePddl.Request:
        return GeneratePddl.Request()

    def response_handler(
        self, blackboard: Blackboard, response: GeneratePddl.Response
    ) -> str:
        blackboard["domain"] = response.domain
        blackboard["problem"] = response.problem
        blackboard["result"].generate_pddl = True

        self._node.get_logger().info(response.domain)
        self._node.get_logger().info(response.problem)

        return SUCCEED
