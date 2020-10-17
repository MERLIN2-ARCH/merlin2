
""" Merlin2 Executor Node """

import asyncio
import collections
import threading

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from merlin2_plan_sys_interfaces.srv import (
    GeneratePddl, Plan, ParsePlan
)
from merlin2_plan_sys_interfaces.action import DispatchPlan, Execute


class Merlin2ExecutorNode(Node):
    """ Merlin2 Executor Node Class """

    def __init__(self):

        super().__init__('merlin2_executor_mode')

        # service clients
        self.__pddl_generator_client = self.create_client(
            GeneratePddl, "generate_pddl")
        self.__plan_client = self.create_client(
            Plan, "plan")

        # action server
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self._action_server = ActionServer(self,
                                           Execute,
                                           "execute",
                                           execute_callback=self.__execute_server,
                                           cancel_callback=self.__cancel_server,
                                           handle_accepted_callback=self.__accepted_callback,
                                           )

    def destroy(self):
        """ destroy node method
        """

        self._action_server.destroy()
        super().destroy_node()

    def __execute_server(self, goal_handle):
        """ execute action server

        Args:
            goal_handle ([type]): goal_handle
        """

        try:
            result = Execute.Result()

            pddl_generated = asyncio.run(self.generate_pddl())
            self.get_logger().info(pddl_generated.domain)
            self.get_logger().info(pddl_generated.problem)
            result.pddl_generator = True

            plan = asyncio.run(
                self.plan(pddl_generated.domain, pddl_generated.problem))
            self.get_logger().info(plan.plan)
            result.planner = True

            goal_handle.succeed()

            return result

        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info("Next goal pulled from the queue")
                    self._current_goal.execute()

                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None

    def __accepted_callback(self, goal_handle):
        """ action server accepted callback or defer execution of an already accepted goal

        Args:
            goal_handle ([type]): goal habdle
        """

        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info("Goal put in the queue")
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __cancel_server(self, goal_handle):
        """ cancelling action server

        Args:
            goal_handle ([type]): goal_handle

        Returns:
            [type]: CancelResponse
        """

        self.get_logger().info("cancelling action server")
        return CancelResponse.ACCEPT

    async def generate_pddl(self) -> GeneratePddl.Response:
        """ asyn generate pddl method

        Returns:
            GeneratePddl.Response: pddl (domain and problem)
        """

        req = GeneratePddl.Request()
        self.__pddl_generator_client.wait_for_service()
        future = self.__pddl_generator_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

        return future.result()

    async def plan(self, domain: str, problem: str) -> Plan.Response:
        """ asyn plan method

        Args:
            domain (str): str of pddl domain
            problem (str): str of pddl problem

        Returns:
            Plan.Response: Plan.Response: pddl (plan)
        """

        req = Plan.Request()
        req.domain = domain
        req.problem = problem
        self.__plan_client.wait_for_service()
        future = self.__plan_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2ExecutorNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    node.destroy()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
