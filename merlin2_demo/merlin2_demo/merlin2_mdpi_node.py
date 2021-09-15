

import time
from random import seed, randint
import threading
import rclpy

from merlin2_mission import Merlin2MissionNode

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
)

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)


from .pddl import wp_checked


class Merlin2MdpiNode(Merlin2MissionNode):

    def __init__(self):

        super().__init__("mdpi_node", run_mission=False)

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

    def create_objects(self):
        self.wp0 = PddlObjectDto(wp_type, "wp0")
        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")
        wp3 = PddlObjectDto(wp_type, "wp3")
        objects = [self.wp0, wp1, wp2, wp3]
        return objects

    def create_propositions(self):
        robot_at_prop = PddlPropositionDto(robot_at, [self.wp0])
        propositions = [robot_at_prop]
        return propositions

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
        pddl_proposition_dao = self.dao_factory.create_pddl_proposition_dao()

        start_t = time.time()

        while self.wp_list:

            # get next wp
            wp = self.wp_list.pop()
            cancel = wp["cancel"]
            wp = PddlObjectDto(wp_type, wp["value"])
            goal = PddlPropositionDto(wp_checked, [wp], is_goal=True)

            # send goal
            thread = threading.Thread(target=self.execute_goal, args=(goal,))
            thread.start()

            # cancel?
            if cancel:
                self.get_logger().info(
                    "CANCELING ------------------------------------")
                i = 0
                while i < self.time_to_cancel and thread.is_alive():
                    time.sleep(1)
                    i += 1
                self.cancel_goals()

            # wait for thread
            thread.join()

        # results
        end_t = time.time()
        total_t = end_t - start_t
        self.get_logger().info("TIME: " + str(total_t) +
                               " ------------------------------------")


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2MdpiNode()

    node.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
