
""" MERLIN2 action that simulates checking a wp """

from typing import List
import rclpy

from pddl_dto import (
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

from ros2_text_to_speech_interfaces.action import TTS
from ros2_text_to_speech_interfaces.msg import Config

from merlin2_arch_interfaces.msg import PlanAction

from .pddl import wp_checked


class Merlin2CheckWpAction(Merlin2Action):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

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
        condition_1 = PddlConditionEffectDto(robot_at,
                                             [self.__wp],
                                             time=PddlConditionEffectDto.AT_START)

        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(wp_checked,
                                          [self.__wp],
                                          time=PddlConditionEffectDto.AT_END)

        return [effect_1]


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2CheckWpAction()

    node.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
