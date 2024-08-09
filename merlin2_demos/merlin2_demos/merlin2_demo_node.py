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

from merlin2_mission import Merlin2MissionNode

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
    person_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
    person_at
)

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from merlin2_demos.pddl import person_attended


class Merlin2DemoNode(Merlin2MissionNode):

    def __init__(self) -> None:
        super().__init__("demo_node")

    def create_objects(self):
        kitchen = PddlObjectDto(wp_type, "kitchen")
        bedroom = PddlObjectDto(wp_type, "bedroom")
        self.livingroom = PddlObjectDto(wp_type, "livingroom")
        self.entrance = PddlObjectDto(wp_type, "entrance")
        bathroom = PddlObjectDto(wp_type, "bathroom")
        self.miguel = PddlObjectDto(person_type, "miguel")
        objects = [kitchen, bedroom, self.livingroom,
                   self.entrance, bathroom, self.miguel]
        return objects

    def create_propositions(self):
        miguel_at_prop = PddlPropositionDto(
            person_at, [self.miguel, self.livingroom])
        robot_at_prop = PddlPropositionDto(robot_at, [self.entrance])
        return [robot_at_prop, miguel_at_prop]

    def execute_mission(self):
        self.get_logger().info("EXECUTING MISSION")
        person_attended_goal = PddlPropositionDto(
            person_attended, [self.miguel], is_goal=True)
        succeed = self.execute_goal(person_attended_goal)
        self.get_logger().info(str(succeed))


def main():
    rclpy.init()
    Merlin2DemoNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
