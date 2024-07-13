#!/usr/bin/env python3

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


""" Merlin2 Planner Node """

import time
import rclpy
from rclpy.node import Node

from merlin2_msgs.srv import GeneratePlan
from merlin2_planner.merlin2_planner_factory import Merlin2PlannerFactory
from merlin2_planner import Merlin2Planners


class Merlin2PlannerNode(Node):
    """ Merlin2 Planner Node Class """

    def __init__(self) -> None:

        super().__init__("planner_node", namespace="merlin2")

        planner_factory = Merlin2PlannerFactory()

        # param names
        planner_param_name = "planner"

        # declaring params
        self.declare_parameter(planner_param_name,
                               Merlin2Planners.POPF)

        # getting params
        planner = self.get_parameter(
            planner_param_name).get_parameter_value().integer_value

        # creating planner
        self.planner = planner_factory.create_planner(planner)

        # service servers
        self.__planner_service = self.create_service(
            GeneratePlan, "generate_plan", self.__planner_srv)

        self.get_logger().info("Planner Started")

    def __planner_srv(
            self,
            req: GeneratePlan.Request,
            res: GeneratePlan.Response
    ) -> GeneratePlan.Response:
        """ plan srv callback

        Args:
            req (GeneratePlan.Request): request (domain and problem)
            res (GeneratePlan.Response): response (plan)

        Returns:
            GeneratePlan.Response: response (plan)
        """

        start_time = time.time()
        self.planner.generate_plan(req.domain, req.problem)
        res.has_solution = self.planner.has_solution()
        res.plan = self.planner.get_plan_actions()
        self.get_logger().info(self.planner.get_str_plan())

        end_time = time.time()

        elapsed_time = end_time - start_time
        self.get_logger().info(
            f"Time to plan: {elapsed_time} seconds")

        return res


def main():
    rclpy.init()
    node = Merlin2PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
