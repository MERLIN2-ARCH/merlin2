
""" Navigation State """

from ros2_text_to_speech_interfaces.action import TTS
from ros2_speech_recognition_interfaces.action import ListenOnce
from ros2_fsm.ros2_states import AcionState, BasicOutomes
from ros2_fsm.basic_fsm import StateMachine
from ros2_fsm.basic_fsm.blackboard import Blackboard


class Merlin2SttState(StateMachine):
    """ Navigation State Class """

    def __init__(self, node):

        super().__init__(
            [BasicOutomes.SUCC, BasicOutomes.ABOR, BasicOutomes.CANC])

        tts_state = AcionState(
            node, TTS, "/text_to_speech/tts", self.create_tts_goal)

        stt_state = AcionState(
            node, ListenOnce, "/speech_recognition/listen_once", self.create_stt_goal,
            outcomes=["repeat"],
            resutl_handler=self.result_stt_handler)

        self.add_state(
            "STT",
            stt_state,
            {"repeat": "TTS"})

        self.add_state(
            "TTS",
            tts_state,
            {BasicOutomes.SUCC: "STT"})

    def create_tts_goal(self, blackboard: Blackboard) -> TTS.Goal:
        """ create a goal for the tts system

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            TTS.Goal: goal
        """

        goal = TTS.Goal()
        goal.text = "I didn't understand you"
        return goal

    def create_stt_goal(self, blackboard: Blackboard) -> ListenOnce.Goal:
        """ create a goal for the stt system

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            ListenOnce.Goal: goal
        """

        goal = ListenOnce.Goal()
        return goal

    def result_stt_handler(self, blackboard: Blackboard, result: ListenOnce.Result) -> str:
        """ handle STT results

        Args:
            blackboard (Blackboard): blackboard of the fsm
            result (ListenOnce.Result): stt results

        Returns:
            str: outcome
        """

        if len(result.stt_strings) != 0:
            blackboard.speech = result.stt_strings
            return BasicOutomes.SUCC

        else:
            return "repeat"
