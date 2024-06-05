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
import time

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at
)

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)
from yasmin import CbState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard

from merlin2_demo.pddl import door_checked, door_at, door_type

doorbell_sounds = ['Doorbell', 'Bell', 'Ding-dong',
                   'Tubular bells', 'Reversing beeps', 'Beep, bleep']


class Merlin2CheckDoorFsmAction(Merlin2FsmAction):
    """ Merlin2 Check Door Action Class """

    def __init__(self) -> None:

        self.__entrance = PddlObjectDto(wp_type, "wp")
        self.__door = PddlObjectDto(door_type, "door")

        super().__init__("check_door")

        prepare_goal_check_state = CbState(["valid"], self.prepare_check_door)
        prepare_goal_welcome_state = CbState(["valid"], self.prepare_welcome)

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "PREPARING_CHECK_THE_DOOR",
            prepare_goal_check_state,
            {"valid": "CHECKING_THE_DOOR"}
        )

        self.add_state(
            "CHECKING_THE_DOOR",
            tts_state,
            {SUCCEED: "PREPARING_WELCOME"}
        )

        self.add_state(
            "PREPARING_WELCOME",
            prepare_goal_welcome_state,
            {"valid": "WELCOME"}
        )

        self.add_state(
            "WELCOME",
            tts_state
        )

    def prepare_check_door(self, blackboard: Blackboard) -> str:
        blackboard.text = "Welcome, open the door"
        return "valid"

    def prepare_welcome(self, blackboard: Blackboard) -> str:
        time.sleep(4)
        blackboard.text = "Hi, come with me to the living room"
        return "valid"

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__entrance, self.__door]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        condition_1 = PddlConditionEffectDto(
            door_at,
            [self.__door, self.__entrance],
            time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            robot_at,
            [self.__entrance],
            time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            door_checked,
            [self.__door],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]


def main():
    rclpy.init()
    node = Merlin2CheckDoorFsmAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
