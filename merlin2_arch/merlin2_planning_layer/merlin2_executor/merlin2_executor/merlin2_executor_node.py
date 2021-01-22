
""" Merlin2 Executor Node """


import time
import rclpy

from merlin2_arch_interfaces.srv import (
    GeneratePddl, GeneratePlan
)
from merlin2_arch_interfaces.action import DispatchPlan, Execute

from custom_ros2 import (
    Node,
    ActionSingleServer,
    ActionClient
)


class Merlin2ExecutorNode(Node):
    """ Merlin2 Executor Node Class """

    def __init__(self):

        super().__init__('merlin2_executor_mode')

        self.__server_canceled = False

        # service clients
        self.__pddl_generator_client = self.create_client(
            GeneratePddl, "generate_pddl")
        self.__planner_client = self.create_client(
            GeneratePlan, "generate_plan")

        # action client
        self.__plan_dispatcher_client = ActionClient(
            self, DispatchPlan, "dispatch_plan")

        # action server
        self.__action_server = ActionSingleServer(self,
                                                  Execute,
                                                  "execute",
                                                  execute_callback=self.__execute_server,
                                                  cancel_callback=self.__cancel_callback
                                                  )

    def destroy(self):
        """ destroy node method """

        self.__action_server.destroy()
        super().destroy_node()

    def __cancel_callback(self):
        self.__server_canceled = True
        if self.__plan_dispatcher_client.is_working():
            self.__plan_dispatcher_client.cancel_goal()

    def __execute_server(self, goal_handle):
        """ execute action server

        Args:
            goal_handle: goal_handle
        """

        self.__server_canceled = False
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

        if not self.__server_canceled:
            # Plan Dispatcher
            goal = DispatchPlan.Goal()
            goal.plan = plan.plan
            self.__plan_dispatcher_client.wait_for_server()
            self.__plan_dispatcher_client.send_goal(goal)
            self.__plan_dispatcher_client.wait_for_result()
            self.get_logger().info(str(self.__plan_dispatcher_client.get_status()))
            result.dispatch_plan = self.__plan_dispatcher_client.is_succeeded()

        if not self.__server_canceled:
            goal_handle.succeed()

        else:
            while not goal_handle.is_cancel_requested:
                time.sleep(0.05)
            goal_handle.canceled()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2ExecutorNode()

    node.join_spin()

    node.destroy()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
