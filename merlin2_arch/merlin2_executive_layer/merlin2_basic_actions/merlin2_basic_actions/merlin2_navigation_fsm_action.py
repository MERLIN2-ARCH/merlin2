
""" MERLIN2 action that uses the topological navigation """

from typing import List
import rclpy

from pddl_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)
from ros2_fsm.basic_fsm import CbState


class Merlin2NavigationAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

        self.__org = PddlObjectDto(wp_type, "o")
        self.__dst = PddlObjectDto(wp_type, "d")

        super().__init__("navigation")

        prepare_goal_state = CbState(["valid"], self.prepapre_goal)

        navigation_state = self.create_state(Merlin2BasicStates.NAVIGATION)

        self.add_state(
            "PREPARING_GOAL",
            prepare_goal_state,
            {"valid": "NAVIGATING"}
        )

        self.add_state(
            "NAVIGATING",
            navigation_state
        )

    def prepapre_goal(self, blackboard):
        blackboard.destination = blackboard.merlin2_action_goal.objects[1]
        return "valid"

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
