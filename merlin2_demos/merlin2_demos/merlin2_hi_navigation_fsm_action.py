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
)

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
    person_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
    person_at
)

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)
from yasmin import CbState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard

from merlin2_demos.pddl import person_attended


class Merlin2HiNavigationFsmAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self) -> None:

        self.__source_p = PddlObjectDto(wp_type, "source_p")
        self.__per = PddlObjectDto(person_type, "per")

        super().__init__("hi_navigation")

        prepapre_question_state = CbState(["valid"], self.prepapre_question)
        check_stt_state = CbState(["valid", "repeat"], self.check_stt)
        tts_state = self.create_state(Merlin2BasicStates.TTS)
        stt_state = self.create_state(Merlin2BasicStates.STT)
        navigation_state = self.create_state(Merlin2BasicStates.NAVIGATION)

        self.add_state(
            "PREPARING_QUESTION",
            prepapre_question_state,
            {"valid": "ASKING"}
        )

        self.add_state(
            "ASKING",
            tts_state,
            {SUCCEED: "LISTENING"}
        )

        self.add_state(
            "LISTENING",
            stt_state,
            {SUCCEED: "CHECKING_SPEECH"}
        )

        self.add_state(
            "CHECKING_SPEECH",
            check_stt_state,
            {
                "valid": "NAVIGATING",
                "repeat": "PREPARING_QUESTION"
            }
        )

        self.add_state(
            "NAVIGATING",
            navigation_state
        )

    def prepapre_question(self, blackboard: Blackboard) -> str:
        blackboard["text"] = "Where do you want me to go?"
        return "valid"

    def check_stt(self, blackboard: Blackboard) -> str:
        if blackboard["speech"][0] == "go":
            blackboard["destination"] = blackboard["speech"][1]
            return "valid"

        return "repeat"

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__source_p, self.__per]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(
            robot_at,
            [self.__source_p],
            time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            person_at,
            [self.__per, self.__source_p],
            time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            person_attended,
            [self.__per],
            time=PddlConditionEffectDto.AT_END
        )

        effect_2 = PddlConditionEffectDto(
            robot_at,
            [self.__source_p],
            is_negative=True,
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1, effect_2]


def main():
    rclpy.init()
    node = Merlin2HiNavigationFsmAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
