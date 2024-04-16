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


""" MERLIN2 action that simulates checking a wp """

from typing import List
import rclpy

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at
)

from merlin2_action.merlin2_action import Merlin2Action

from text_to_speech_msgs.action import TTS
from text_to_speech_msgs.msg import Config

from merlin2_msgs.msg import PlanAction

from merlin2_demo.pddl import wp_checked


class Merlin2CheckWpAction(Merlin2Action):
    """ Merlin2 Navigation Action Class """

    def __init__(self) -> None:

        self.__wp = PddlObjectDto(wp_type, "wp")

        super().__init__("check_wp")

        self.__tts_client = self.create_action_client(
            TTS, "/text_to_speech/tts")

    def run_action(self, goal: PlanAction) -> bool:
        tts_goal = TTS.Goal()

        tts_goal.text = "Waypoint " + str(goal.objects[0][-1]) + " checked."
        tts_goal.config.tool = Config.GTTS

        self.__tts_client.wait_for_server()
        self.__tts_client.send_goal(tts_goal)
        self.__tts_client.wait_for_result()

        if not self.__tts_client.is_succeeded():
            return False

        return True

    def cancel_action(self):
        self.__tts_client.cancel_goal()

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(
            robot_at,
            [self.__wp],
            time=PddlConditionEffectDto.AT_START
        )

        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            wp_checked,
            [self.__wp],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]


def main():
    rclpy.init()
    node = Merlin2CheckWpAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
