
""" Merlin2 Executor Node """


import rclpy

from kant_dao import ParameterLoader

from yasmin import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from merlin2_arch_interfaces.action import Execute

from simple_node import Node

from .states import (
    Merlin2GeneratePddlState,
    Merlin2GeneratePlanState,
    Merlin2DispatchPlanState
)


class Merlin2ExecutorNode(Node, StateMachine):
    """ Merlin2 Executor Node Class """

    def __init__(self):

        Node.__init__(self, "executor_node", namespace="merlin2")
        StateMachine.__init__(self, [SUCCEED, ABORT, CANCEL])

        self.dao_factory = ParameterLoader(self).get_dao_factory()

        # create fsm
        self.add_state("GENERATING_PDDL", Merlin2GeneratePddlState(self),
                       {SUCCEED: "GENERATING_PLAN"})

        self.add_state("GENERATING_PLAN", Merlin2GeneratePlanState(self),
                       {SUCCEED: "DISPATCHING_PLAN"})

        self.add_state("DISPATCHING_PLAN", Merlin2DispatchPlanState(self))

        YasminViewerPub(self, "MERLIN2_EXECUTOR", self)

        # action server
        self.__action_server = self.create_action_server(Execute,
                                                         "execute",
                                                         execute_callback=self.__execute_server,
                                                         cancel_callback=self.__cancel_callback
                                                         )

    def __cancel_callback(self):
        StateMachine.cancel_state(self)

    def delete_goals(self):
        pddl_proposition_dao = self.dao_factory.create_pddl_proposition_dao()

        for pddl_goal_dto in pddl_proposition_dao.get_goals():
            pddl_proposition_dao.delete(pddl_goal_dto)

    def __execute_server(self, goal_handle):
        """ execute action server

        Args:
            goal_handle: goal_handle
        """

        result = Execute.Result()
        result.generate_pddl = False
        result.generate_plan = False
        result.dispatch_plan = False

        blackboard = Blackboard()
        blackboard.result = result

        outcome = self(blackboard)

        if outcome == CANCEL:
            goal_handle.canceled()

        elif outcome == ABORT:
            goal_handle.abort()

        elif outcome == SUCCEED:
            goal_handle.succeed()

        # remove goals
        self.get_logger().info("Removing goals")
        self.delete_goals()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2ExecutorNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
