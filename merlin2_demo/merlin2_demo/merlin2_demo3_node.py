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


import rclpy
import time

from merlin2_mission import Merlin2MissionNode

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at
)

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from merlin2_demo.pddl import door_checked, door_at, door_type, sound_type, sound_listened

sounds_acepted = ['doorbell', 'bell', 'ding-dong',
                  'tubular_bells', 'reversing_beeps', 'beepbleep', 'chime']


class Merlin2Demo3Node(Merlin2MissionNode):

    def __init__(self) -> None:
        super().__init__("demo_3_node")

    def create_objects(self):
        self.livingroom = PddlObjectDto(wp_type, "livingroom")
        self.entrance = PddlObjectDto(wp_type, "entrance")
        self.door = PddlObjectDto(door_type, "d")
        self.doorbell = PddlObjectDto(sound_type, "doorbell")
        objects = [self.livingroom, self.entrance, self.door, self.doorbell]
        return objects

    def create_propositions(self):
        robot_at_prop = PddlPropositionDto(robot_at, [self.livingroom])
        door_at_prop = PddlPropositionDto(door_at, [self.door, self.entrance])
        return [robot_at_prop, door_at_prop]

    def execute_mission(self):
        self.get_logger().info("Waiting for doorbell...")
        list_sounds = []
        # while sound not in sounds_acepted:
        while not (any(x in list_sounds for x in sounds_acepted)):
            list_sounds = []
            sound_listened_prop = self.pddl_proposition_dao.get_by_predicate(
                sound_listened.get_name())

            if sound_listened_prop:
                for i in range(len(sound_listened_prop)):
                    sound = sound_listened_prop[i].get_objects()[0].get_name()
                    list_sounds.append(sound)

        self.get_logger().info("EXECUTING MISSION: Answer the door")
        door_checked_goal = PddlPropositionDto(
            door_checked, [self.door], is_goal=True)
        robot_at_goal = PddlPropositionDto(
            robot_at, [self.livingroom], is_goal=True)

        start = time.time()
        succeed = self.execute_goals([door_checked_goal, robot_at_goal])
        self.get_logger().info(str(succeed))
        end = time.time()
        self.get_logger().info("Execution time: "+str(end-start))


def main():
    rclpy.init()
    Merlin2Demo3Node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
