
""" MERLIN2 action that uses the topological navigation """

from typing import List
import rclpy

from pddl_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from ros2_topological_nav_interfaces.action import TopoNav
from merlin2_fsm_action import Merlin2FsmAction
from ros2_fsm.ros2_states import AcionState


class Merlin2NavigationAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

        self.__org = PddlObjectDto(wp_type, "o")
        self.__dst = PddlObjectDto(wp_type, "d")

        super().__init__("navigation")

        navigation_state = AcionState(
            self, TopoNav, "/topo_nav/navigation", self.create_nav_goal)

        self.add_state(
            "NAV_STATE",
            navigation_state)

    def create_nav_goal(self, blackboard):
        dst = blackboard.merlin2_action_goal.objects[1]
        goal = TopoNav.Goal()
        goal.point = dst
        return goal

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__org, self.__dst]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(robot_at,
                                             [self.__org],
                                             time=PddlConditionEffectDto.AT_START)
        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(robot_at,
                                          [self.__dst],
                                          time=PddlConditionEffectDto.AT_END)

        effect_2 = PddlConditionEffectDto(robot_at,
                                          [self.__org],
                                          is_negative=True,
                                          time=PddlConditionEffectDto.AT_START)

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2NavigationAction()

    node.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
