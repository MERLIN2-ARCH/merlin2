
""" MERLIN2 action that uses the topological navigation """

from typing import List
import rclpy

from pddl_dto import (
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

from ros2_topological_nav_interfaces.action import TopoNav
from ros2_speech_recognition_interfaces.action import ListenOnce
from ros2_text_to_speech_interfaces.action import TTS
from merlin2_arch_interfaces.msg import PlanAction
from custom_ros2 import ActionClient

from .pddl import person_attended


class Merlin2HiNavigationAction(Merlin2Action):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

        self.__source_p = PddlObjectDto(wp_type, "source_p")
        self.__per = PddlObjectDto(person_type, "per")

        super().__init__("hi_navigation_action")

        self.__topo_nav_client = ActionClient(
            self, TopoNav, "/topo_nav/navigation")

        self.__tts_client = ActionClient(
            self, TTS, "/text_to_speech/tts")

        self.__speech_rec_client = ActionClient(
            self, ListenOnce, "/speech_recognition/listen_once")

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
        results = self.__speech_rec_client.get_result()

        if not self.__speech_rec_client.is_succeeded():
            return False

        # navigation
        nav_goal.point = results.stt_strings[1]
        self.__topo_nav_client.wait_for_server()
        self.__topo_nav_client.send_goal(nav_goal)
        self.__topo_nav_client.wait_for_result()

        if not self.__topo_nav_client.is_succeeded():
            return False

        return True

    def cancel_action(self):
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

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
