
import os
import time
from random import seed, randint
import threading
import rclpy

from merlin2_mission import Merlin2FsmMissionNode

# states
from yasmin import CbState
from yasmin.blackboard import Blackboard

# distance measure
from math import sqrt, pow
from geometry_msgs.msg import PoseWithCovarianceStamped

# pddl
from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
)
from .pddl import wp_checked


class Merlin2MdpiNode(Merlin2FsmMissionNode):

    END = "end"
    NEXT = "next"

    def __init__(self):

        super().__init__("mdpi_node", run_mission=False, outcomes=[self.END])

        # parameters
        total_points_param_name = "total_points"
        time_to_cancel_param_name = "time_to_cancel"
        number_of_tests_param_name = "number_of_tests"
        results_path_param_name = "results_path"
        world_param_name = "world"

        self.declare_parameter(
            total_points_param_name, 6)  # 6, 20, 120
        self.declare_parameter(
            time_to_cancel_param_name, 10)
        self.declare_parameter(
            number_of_tests_param_name, 10)
        self.declare_parameter(
            results_path_param_name, "~/")
        self.declare_parameter(
            world_param_name, "granny")

        self.total_points = self.get_parameter(
            total_points_param_name).get_parameter_value().integer_value
        self.time_to_cancel = self.get_parameter(
            time_to_cancel_param_name).get_parameter_value().integer_value
        self.number_of_tests = self.get_parameter(
            number_of_tests_param_name).get_parameter_value().integer_value
        self.results_path = self.get_parameter(
            results_path_param_name).get_parameter_value().string_value
        self.world = self.get_parameter(
            world_param_name).get_parameter_value().string_value

        self.__last_pose = None
        self.__distance = 0

        self.__pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.__pose_cb, 100)

        # build fsm
        self.add_state("INITIATING_BLACKBOARD",
                       CbState([self.END], self.init_blackboard),
                       {self.END: "CHECKING_NEXT_TEST"})

        self.add_state("CHECKING_NEXT_TEST",
                       CbState([self.NEXT, self.END], self.check_next_test),
                       {self.NEXT: "INITIATING_POINTS", self.END: "SAVING_RESULTS"})

        self.add_state("INITIATING_POINTS",
                       CbState([self.END], self.init_points),
                       {self.END: "RUNNING_TEST"})

        self.add_state("RUNNING_TEST",
                       CbState([self.END], self.run_test),
                       {self.END: "CHECKING_NEXT_TEST"})

        self.add_state("SAVING_RESULTS",
                       CbState([self.END], self.save_results))

    def __pose_cb(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose

        if not self.__last_pose is None:
            new_distance = sqrt(
                pow((pose.position.x - self.__last_pose.position.x), 2) +
                pow((pose.position.y - self.__last_pose.position.y), 2))
            self.__distance += new_distance

        self.__last_pose = pose

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

    def init_blackboard(self, blackboard: Blackboard) -> str:
        blackboard.results = []
        return self.END

    def check_next_test(self, blackboard: Blackboard) -> str:

        if len(blackboard.results) >= self.number_of_tests:
            return self.END

        return self.NEXT

    def init_points(self, blackboard: Blackboard) -> str:
        wp_list = []
        seed(time.time())

        for _ in range(self.total_points):
            point = {
                "value": "",
                "cancel": False
            }
            point["value"] = "wp" + str(randint(0, 3))

            if wp_list:
                while point["value"] == wp_list[-1]["value"]:
                    point["value"] = "wp" + str(randint(0, 3))

            wp_list.append(point)

        point_pos_list = []
        for _ in range(int(self.total_points/2)):
            point_pos = randint(0, self.total_points - 1)

            while point_pos in point_pos_list:
                point_pos = randint(0, self.total_points - 1)

            point_pos_list.append(point_pos)
            wp_list[point_pos]["cancel"] = True

        self.get_logger().info(str(wp_list))
        blackboard.wp_list = wp_list

        return self.END

    def run_test(self, blackboard: Blackboard) -> str:

        start_t = time.time()
        self.__distance = 0

        while blackboard.wp_list:

            # get next wp
            wp = blackboard.wp_list.pop()
            cancel = wp["cancel"]
            wp = PddlObjectDto(wp_type, wp["value"])
            goal = PddlPropositionDto(wp_checked, [wp], is_goal=True)

            # send goal
            thread = threading.Thread(target=self.execute_goal, args=(goal,))
            thread.start()

            # cancel?
            if cancel:
                self.get_logger().info("CANCELING " + "-"*40)
                i = 0
                while i < self.time_to_cancel and thread.is_alive():
                    time.sleep(1)
                    i += 1
                self.cancel_goals()

            # wait for thread
            thread.join()

            # remove propositions achieved
            goal.set_is_goal(False)
            self.pddl_proposition_dao.delete(goal)

        # results
        end_t = time.time()
        total_t = end_t - start_t

        blackboard.results.append([total_t, self.__distance])

        return self.END

    def save_results(self, blackboard: Blackboard) -> str:
        string_csv = "ID, World, Time (Seconds), Distance (Meters)\n"

        for i in range(len(blackboard.results)):
            time_t, distance = blackboard.results[i]
            string_csv += str(i) + "," + self.world + "," + \
                str(time_t) + "," + str(distance) + "\n"

        file_name = self.results_path + "/results_" + \
            self.world + "_" + str(self.total_points) + ".csv"
        file_name = file_name.replace("//", "/")
        file_name = os.path.abspath(os.path.expanduser(file_name))

        f = open(file_name, "w")
        f.write(string_csv)
        f.close()

        return self.END


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2MdpiNode()
    node.execute_mission()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
