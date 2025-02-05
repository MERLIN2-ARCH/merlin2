#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta

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


"""MERLIN2 action that uses the waypoint navigation"""

from typing import List

import rclpy

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_demos.pddl import wp_checked
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fsm_action import Merlin2FsmAction, Merlin2BasicStates

from yasmin import CbState
from yasmin.blackboard import Blackboard


class Merlin2CheckWpFsmAction(Merlin2FsmAction):
    """Merlin2 Navigation Action Class"""

    def __init__(self) -> None:

        self.__wp = PddlObjectDto(wp_type, "wp")

        super().__init__("check_wp")

        prepare_goal_state = CbState(["valid"], self.prepapre_goal)

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state("PREPARING_GOAL", prepare_goal_state, {"valid": "CHECKING_WP"})

        self.add_state("CHECKING_WP", tts_state)

    def prepapre_goal(self, blackboard: Blackboard) -> str:
        blackboard["text"] = (
            "Waypoint "
            + str(blackboard["merlin2_action_goal"].objects[0][-1])
            + " checked."
        )
        return "valid"

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(
            robot_at, [self.__wp], time=PddlConditionEffectDto.AT_START
        )

        return [condition_1]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            wp_checked, [self.__wp], time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]


def main():
    rclpy.init()
    node = Merlin2CheckWpFsmAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
