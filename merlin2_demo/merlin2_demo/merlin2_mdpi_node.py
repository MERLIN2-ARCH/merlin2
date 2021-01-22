

import time
from random import seed, randint
import threading
import rclpy

from merlin2_goal_dispatcher import Merlin2GoalDispatcher

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
)

from pddl_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from custom_ros2 import Node

from .pddl import wp_checked


class Merlin2MdpiNode(Node):

    def __init__(self):

        super().__init__("mdpi_node")

        self.goal_dispatcher = Merlin2GoalDispatcher(self)
        self.pddl_dao_factory = self.goal_dispatcher.get_pddl_factory()

        # parameters
        total_points_param_name = "total_points"
        time_to_cancel_param_name = "time_to_cancel"

        self.declare_parameter(
            total_points_param_name, 20)
        self.declare_parameter(
            time_to_cancel_param_name, 10)

        self.total_points = self.get_parameter(
            total_points_param_name).get_parameter_value().integer_value
        self.time_to_cancel = self.get_parameter(
            time_to_cancel_param_name).get_parameter_value().integer_value
        self.wp_list = []

        # initial pddl
        pddl_object_dao = self.pddl_dao_factory.create_pddl_object_dao()
        pddl_proposition_dao = self.pddl_dao_factory.create_pddl_proposition_dao()

        wp0 = PddlObjectDto(wp_type, "wp0")
        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")
        wp3 = PddlObjectDto(wp_type, "wp3")
        self.anywhere = PddlObjectDto(wp_type, "anywhere")
        objects = [wp0, wp1, wp2, wp3, self.anywhere]

        robot_at_prop = PddlPropositionDto(robot_at, [wp0])
        propositions = [robot_at_prop]

        pddl_object_dao.delete_all()
        pddl_proposition_dao.delete_all()

        for pddl_object_dto in objects:
            pddl_object_dao.save(pddl_object_dto)

        for pddl_proposition_dto in propositions:
            pddl_proposition_dao.save(pddl_proposition_dto)

    def init_points(self):
        self.wp_list = []
        seed(time.time())

        for _ in range(self.total_points):
            point = {
                "value": "",
                "cancel": False
            }
            point["value"] = "wp" + str(randint(0, 3))

            if self.wp_list:
                while point["value"] == self.wp_list[-1]["value"]:
                    point["value"] = "wp" + str(randint(0, 3))

            self.wp_list.append(point)

        point_pos_list = []
        for _ in range(int(self.total_points/2)):
            point_pos = randint(0, self.total_points - 1)

            while point_pos in point_pos_list:
                point_pos = randint(0, self.total_points - 1)

            point_pos_list.append(point_pos)
            self.wp_list[point_pos]["cancel"] = True

        print(self.wp_list)

    def execute(self):

        self.init_points()
        pddl_proposition_dao = self.pddl_dao_factory.create_pddl_proposition_dao()

        start_t = time.time()

        while self.wp_list:

            # get next wp
            wp = self.wp_list.pop()
            cancel = wp["cancel"]
            wp = PddlObjectDto(wp_type, wp["value"])
            goal = PddlPropositionDto(wp_checked, [wp], is_goal=True)

            # send goal
            thread = threading.Thread(target=self._check_wp, args=(goal,))
            thread.start()

            # cancel?
            if cancel:
                i = 0
                while i < self.time_to_cancel and thread.is_alive():
                    time.sleep(1)
                    i += 1
                    self.goal_dispatcher.cancel_goals()

            robot_at_prop = pddl_proposition_dao.get_by_predicate(
                robot_at.get_predicate_name())

            if not robot_at_prop:
                robot_at_prop = PddlPropositionDto(robot_at, [self.anywhere])
                pddl_proposition_dao.save(robot_at_prop)

            # wait for thread
            thread.join()
            pddl_proposition_dao.delete(goal)

        # results
        end_t = time.time()
        total_t = end_t - start_t
        self.get_logger().info("TIME: " + str(total_t))

    def _check_wp(self, goal):
        self.goal_dispatcher.execute_goals([goal])


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2MdpiNode()

    node.execute()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
