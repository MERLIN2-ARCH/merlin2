
""" Navigation State """

from text_to_speech_interfaces.action import TTS
from speech_to_text_interfaces.action import ListenOnce
from std_srvs.srv import Empty
from yasmin_ros import AcionState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin import StateMachine, CbState
from yasmin.blackboard import Blackboard


class Merlin2SttState(StateMachine):
    """ Navigation State Class """

    def __init__(self, node):

        self.node = node

        super().__init__(
            [SUCCEED, ABORT, CANCEL])

        checking_speech_state = CbState(
            ["valid", "repeat"], self.checking_speech)

        calibrating_state = ServiceState(
            node, Empty, "/speech_to_text/calibrate_listening", self.crate_calibrate_rquest)

        tts_state = AcionState(
            node, TTS, "/text_to_speech/tts", self.create_tts_goal)

        stt_state = AcionState(
            node, ListenOnce, "/speech_to_text/listen_once", self.create_stt_goal,
            resutl_handler=self.result_stt_handler)

        self.add_state(
            "CALIBRATING",
            calibrating_state,
            {SUCCEED: "LISTENING"})

        self.add_state(
            "LISTENING",
            stt_state,
            {SUCCEED: "CHECKING_SPEECH"})

        self.add_state(
            "CHECKING_SPEECH",
            checking_speech_state,
            {"repeat": "COMPLAINING",
             "valid": SUCCEED})

        self.add_state(
            "COMPLAINING",
            tts_state,
            {SUCCEED: "CALIBRATING"})

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
        goal.calibrate = False
        return goal

    def checking_speech(self, blackboard: Blackboard) -> str:
        """ check STT results

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            str: outcome
        """

        if len(blackboard.speech) != 0:
            return "valid"

        else:
            return "repeat"

    def crate_calibrate_rquest(self, blackboard: Blackboard) -> Empty.Request:
        """ create a calibreate request

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            Empty.Request: empty request
        """

        return Empty.Request()

    def result_stt_handler(self, blackboard: Blackboard, result: ListenOnce.Result) -> str:
        """ handle STT results

        Args:
            blackboard (Blackboard): blackboard of the fsm
            result (ListenOnce.Result): stt results

        Returns:
            str: outcome
        """

        blackboard.speech = result.stt_strings
        return SUCCEED
