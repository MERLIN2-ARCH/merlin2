#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" MERLIN2 action that uses the waypoint navigation """

from typing import List

import rclpy

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
    PddlPropositionDto
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)
from yasmin import CbState
from yasmin.blackboard import Blackboard

# knowledge after cancel
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_basic_actions.merlin2_basic_types import wp_type


class Merlin2NavigationFsmAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self) -> None:

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

    def cancel_action(self) -> None:

        super().cancel_action()

        anywhere = PddlObjectDto(wp_type, "anywhere")
        prop = PddlPropositionDto(robot_at, [anywhere])

        pddl_proposition_dao = self.dao_factory.create_pddl_proposition_dao()
        pddl_proposition_dao.save(prop)

    def prepapre_goal(self, blackboard: Blackboard) -> str:
        blackboard.destination = blackboard.merlin2_action_goal.objects[1]
        return "valid"

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__org, self.__dst]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(
            robot_at,
            [self.__org],
            time=PddlConditionEffectDto.AT_START
        )
        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(
            robot_at,
            [self.__dst],
            time=PddlConditionEffectDto.AT_END
        )

        effect_2 = PddlConditionEffectDto(
            robot_at,
            [self.__org],
            is_negative=True,
            time=PddlConditionEffectDto.AT_START
        )

        return [effect_1, effect_2]


def main():
    rclpy.init()
    node = Merlin2NavigationFsmAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
