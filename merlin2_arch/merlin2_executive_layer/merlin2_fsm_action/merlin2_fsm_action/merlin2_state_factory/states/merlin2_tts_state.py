# Copyright (C) 2023 Miguel Ángel González Santamarta

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


"""Navigation State"""

from rclpy.node import Node
from text_to_speech_msgs.action import TTS
from yasmin_ros import ActionState
from yasmin.blackboard import Blackboard


class Merlin2TtsState(ActionState):
    """Navigation State Class"""

    def __init__(self, node: Node) -> None:

        super().__init__(TTS, "/text_to_speech/tts", self.create_tts_goal, node=node)

    def create_tts_goal(self, blackboard: Blackboard) -> TTS.Goal:
        """create a goal for the tts system

            blackboard:
                text

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            TTS.Goal: goal
        """

        goal = TTS.Goal()
        goal.text = blackboard["text"]
        return goal
