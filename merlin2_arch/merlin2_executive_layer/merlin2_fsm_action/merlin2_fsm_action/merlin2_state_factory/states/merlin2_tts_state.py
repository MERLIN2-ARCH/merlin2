
""" Navigation State """

from text_to_speech_interfaces.action import TTS
from yasmin_ros import AcionState
from yasmin.blackboard import Blackboard


class Merlin2TtsState(AcionState):
    """ Navigation State Class """

    def __init__(self, node):

        super().__init__(node, TTS, "/text_to_speech/tts", self.create_tts_goal)

    def create_tts_goal(self, blackboard: Blackboard) -> TTS.Goal:
        """ create a goal for the tts system

            blackboard:
                text

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            TTS.Goal: goal
        """

        goal = TTS.Goal()
        goal.text = blackboard.text
        return goal
