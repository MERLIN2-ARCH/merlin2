
""" MERLIN2 action that asks for a wp to move to """

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

from merlin2_action.merlin2_action import Merlin2Action

from topological_nav_interfaces.action import TopoNav
from speech_to_text_interfaces.action import ListenOnce
from text_to_speech_interfaces.action import TTS
from merlin2_arch_interfaces.msg import PlanAction

from .pddl import person_attended


class Merlin2HiNavigationAction(Merlin2Action):
    """ Merlin2 HI Navigation Action Class """

    def __init__(self):

        self.__source_p = PddlObjectDto(wp_type, "source_p")
        self.__per = PddlObjectDto(person_type, "per")

        super().__init__("hi_navigation")

        self.__topo_nav_client = self.create_action_client(
            TopoNav, "/topological_nav/navigation")

        self.__tts_client = self.create_action_client(
            TTS, "/text_to_speech/tts")

        self.__speech_rec_client = self.create_action_client(
            ListenOnce, "/speech_to_text/listen_once")

    def run_action(self, goal: PlanAction) -> bool:
        nav_goal = TopoNav.Goal()
        tts_goal = TTS.Goal()
        speech_recog_goal = ListenOnce.Goal()

        # tts
        tts_goal.text = "Hello. Where do you want me to go?"
        self.__tts_client.wait_for_server()
        self.__tts_client.send_goal(tts_goal)
        self.__tts_client.wait_for_result()

        if not self.__tts_client.is_succeeded():
            return False

        # speech
        self.__speech_rec_client.wait_for_server()
        self.__speech_rec_client.send_goal(speech_recog_goal)
        self.__speech_rec_client.wait_for_result()
        results = self.__speech_rec_client.get_result().stt_strings

        while len(results) != 2 or results[0] != "go":
            self.__speech_rec_client.wait_for_server()
            self.__speech_rec_client.send_goal(speech_recog_goal)
            self.__speech_rec_client.wait_for_result()
            results = self.__speech_rec_client.get_result()

        if not self.__speech_rec_client.is_succeeded():
            return False

        # navigation
        nav_goal.point = results[1]
        self.__topo_nav_client.wait_for_server()
        self.__topo_nav_client.send_goal(nav_goal)
        self.__topo_nav_client.wait_for_result()

        if not self.__topo_nav_client.is_succeeded():
            return False

        return True

    def cancel_action(self):
        self.__tts_client.cancel_goal()
        self.__speech_rec_client.cancel_goal()
        self.__topo_nav_client.cancel_goal()

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__source_p, self.__per]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(robot_at,
                                             [self.__source_p],
                                             time=PddlConditionEffectDto.AT_START)

        condition_2 = PddlConditionEffectDto(person_at,
                                             [self.__per, self.__source_p],
                                             time=PddlConditionEffectDto.AT_START)

        return [condition_1, condition_2]

    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(person_attended,
                                          [self.__per],
                                          time=PddlConditionEffectDto.AT_END)

        effect_2 = PddlConditionEffectDto(robot_at,
                                          [self.__source_p],
                                          is_negative=True,
                                          time=PddlConditionEffectDto.AT_END)

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2HiNavigationAction()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
