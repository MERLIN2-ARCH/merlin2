#!/usr/bin/env python3

""" Merlin2 Planner Node """

import rclpy

from merlin2_msgs.srv import GeneratePlan
from merlin2_planner.merlin2_planner_factory import Merlin2PlannerFactory
from merlin2_planner import Merlin2Planners

from simple_node import Node


class Merlin2PlannerNode(Node):
    """ Merlin2 Planner Node Class """

    def __init__(self):

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

    def __planner_srv(self,
                      req: GeneratePlan.Request,
                      res: GeneratePlan.Response) -> GeneratePlan.Response:
        """ plan srv callback

        Args:
            req (GeneratePlan.Request): request (domain and problem)
            res (GeneratePlan.Response): response (plan)

        Returns:
            GeneratePlan.Response: response (plan)
        """

        self.planner.generate_plan(req.domain, req.problem)
        res.has_solution = self.planner.has_solution()
        res.plan = self.planner.get_plan_actions()
        self.get_logger().info(self.planner.get_str_plan())

        return res


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PlannerNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
