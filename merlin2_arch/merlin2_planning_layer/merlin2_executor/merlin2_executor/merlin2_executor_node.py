
""" Merlin2 Executor Node """


import rclpy

from merlin2_arch_interfaces.srv import (
    GeneratePddl, GeneratePlan
)
from merlin2_arch_interfaces.action import DispatchPlan, Execute

from custom_ros2 import Node


class Merlin2ExecutorNode(Node):
    """ Merlin2 Executor Node Class """

    def __init__(self):

        super().__init__("merlin2_executor_node")

        # service clients
        self.__pddl_generator_client = self.create_client(
            GeneratePddl, "generate_pddl")
        self.__planner_client = self.create_client(
            GeneratePlan, "generate_plan")

        # action client
        self.__plan_dispatcher_client = self.create_action_client(
            DispatchPlan, "dispatch_plan")

        # action server
        self.__action_server = self.create_action_server(Execute,
                                                         "execute",
                                                         execute_callback=self.__execute_server,
                                                         cancel_callback=self.__cancel_callback
                                                         )

    def __cancel_callback(self):
        if self.__plan_dispatcher_client.is_working():
            self.__plan_dispatcher_client.cancel_goal()

    def __execute_server(self, goal_handle):
        """ execute action server

        Args:
            goal_handle: goal_handle
        """

        result = Execute.Result()

        # PDDL Generator
        req = GeneratePddl.Request()
        self.__pddl_generator_client.wait_for_service()
        pddl_generated = self.__pddl_generator_client.call(req)
        self.get_logger().info(pddl_generated.domain)
        self.get_logger().info(pddl_generated.problem)
        result.generate_pddl = True

        # Planner
        req = GeneratePlan.Request()
        req.domain = pddl_generated.domain
        req.problem = pddl_generated.problem
        self.__planner_client.wait_for_service()
        plan = self.__planner_client.call(req)
        self.get_logger().info(str(plan.has_solution))
        self.get_logger().info(str(plan.plan))
        result.generate_plan = True

        if not self.__action_server.is_canceled():
            # Plan Dispatcher
            goal = DispatchPlan.Goal()
            goal.plan = plan.plan
            self.__plan_dispatcher_client.wait_for_server()
            self.__plan_dispatcher_client.send_goal(goal)
            self.__plan_dispatcher_client.wait_for_result()
            self.get_logger().info(str(self.__plan_dispatcher_client.get_status()))
            result.dispatch_plan = self.__plan_dispatcher_client.is_succeeded()

        if self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled()
        else:
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2ExecutorNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
