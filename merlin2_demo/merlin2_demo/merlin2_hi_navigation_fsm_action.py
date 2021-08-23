
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

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)
from ros2_fsm.basic_fsm import CbState
from ros2_fsm.basic_outcomes import SUCCEED

from .pddl import person_attended


class Merlin2HiNavigationAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

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
            {"valid": "NAVIGATING",
             "repeat": "PREPARING_QUESTION"}
        )

        self.add_state(
            "NAVIGATING",
            navigation_state
        )

    def prepapre_question(self, blackboard):
        blackboard.text = "Where do you want me to go?"
        return "valid"

    def check_stt(self, blackboard):
        if blackboard.speech[0] == "go":
            blackboard.destination = blackboard.speech[1]
            return "valid"

        return "repeat"

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


if __name__ == '__main__':
    main()
