
""" Merlin2 Planner Node """

import rclpy
from rclpy.node import Node

from merlin2_plan_sys_interfaces.srv import Plan

from merlin2_planner.merlin2_planner_factory.merlin2_planner_factory import Merlin2PlannerFactory
from merlin2_planner.merlin2_planner_factory.merlin2_planners import Merlin2Planners


class Merlin2PlannerNode(Node):
    """ Merlin2 Planner Node Class """

    def __init__(self):

        super().__init__('merlin2_planner_mode')

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
            Plan, 'plan', self.__planner_srv)

    def __planner_srv(self,
                      req: Plan.Request,
                      res: Plan.Response) -> Plan.Response:
        """ plan srv callback

        Args:
            req (Plan.Request): request (domain and problem)
            res (Plan.Response): response (plan)

        Returns:
            Plan.Response: response (plan)
        """

        res.plan = self.planner.plan(req.domain, req.problem)

        return res


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PlannerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
