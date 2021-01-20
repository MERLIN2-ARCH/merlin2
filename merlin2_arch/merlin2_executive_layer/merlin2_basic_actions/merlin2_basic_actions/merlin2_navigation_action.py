
""" MERLIN2 action that uses the topological navigation """

from typing import List
import rclpy

from pddl_dto import (
    PddlObjectDto,
    PddlPredicateDto,
    PddlConditionEffectDto,
    PddlTypeDto
)

from merlin2_action.merlin2_action import Merlin2Action

from ros2_topological_nav_interfaces.action import TopoNav
from merlin2_arch_interfaces.msg import PlanAction
from custom_ros2 import ActionClient


class Merlin2NavigationAction(Merlin2Action):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

        self.__wp_type = PddlTypeDto("wp")
        self.__robot_at = PddlPredicateDto("robot_at", [self.__wp_type])

        self.__org = PddlObjectDto(self.__wp_type, "o")
        self.__dst = PddlObjectDto(self.__wp_type, "d")

        super().__init__("navigation_action")

        self.__action_client = ActionClient(self, TopoNav, "topo_nav")

    def run_action(self, goal: PlanAction) -> bool:
        nav_goal = TopoNav.Goal()

        dst = goal.objects[1]
        nav_goal.point = dst

        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_succeeded():
            return True

        else:
            return False

    def cancel_action(self):
        self.__action_client.cancel_goal()

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__org, self.__dst]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(self.__robot_at,
                                             [self.__org],
                                             time=PddlConditionEffectDto.AT_START)
        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(self.__robot_at,
                                          [self.__dst],
                                          time=PddlConditionEffectDto.AT_END)

        effect_2 = PddlConditionEffectDto(self.__robot_at,
                                          [self.__org],
                                          is_negative=True,
                                          time=PddlConditionEffectDto.AT_START)

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2NavigationAction()

    node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
