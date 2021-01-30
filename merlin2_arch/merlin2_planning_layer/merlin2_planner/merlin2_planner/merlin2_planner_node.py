
""" Merlin2 Planner Node """

import rclpy

from merlin2_arch_interfaces.srv import GeneratePlan

from merlin2_planner.merlin2_planner_factory import (
    Merlin2PlannerFactory,
    Merlin2Planners
)

from custom_ros2 import Node


class Merlin2PlannerNode(Node):
    """ Merlin2 Planner Node Class """

    def __init__(self):

        super().__init__("merlin2_planner_node")

        planner_factory = Merlin2PlannerFactory()

        # param names
        planner_num_param_name = "planner_num"

        # declaring params
        self.declare_parameter(planner_num_param_name,
                               Merlin2Planners.POPF)

        # getting params
        planner_num = self.get_parameter(
            planner_num_param_name).get_parameter_value().integer_value

        # creating planner
        self.planner = planner_factory.create_planner(planner_num)

        # service servers
        self.__start_server = self.create_service(
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
        res.plan = self.planner.get_actions_plan()

        return res


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PlannerNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
