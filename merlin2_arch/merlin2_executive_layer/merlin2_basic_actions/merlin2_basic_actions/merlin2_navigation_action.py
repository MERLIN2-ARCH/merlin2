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


""" MERLIN2 action that uses the waypoint navigation """

from typing import List
import rclpy

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_action.merlin2_action import Merlin2Action

from waypoint_navigation_msgs.action import NavigateToWp
from merlin2_msgs.msg import PlanAction


class Merlin2NavigationAction(Merlin2Action):
    """ Merlin2 Navigation Action Class """

    def __init__(self):

        self.__org = PddlObjectDto(wp_type, "o")
        self.__dst = PddlObjectDto(wp_type, "d")

        super().__init__("navigation")

        self.__wp_nav_client = self.create_action_client(
            NavigateToWp, "/waypoint_navigation/navigate_to_wp")

    def run_action(self, goal: PlanAction) -> bool:
        nav_goal = NavigateToWp.Goal()

        dst = goal.objects[1]
        nav_goal.wp_id = dst

        self.__wp_nav_client.wait_for_server()
        self.__wp_nav_client.send_goal(nav_goal)
        self.__wp_nav_client.wait_for_result()

        if self.__wp_nav_client.is_succeeded():
            return True

        else:
            return False

    def cancel_action(self):
        self.__wp_nav_client.cancel_goal()

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__org, self.__dst]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(robot_at,
                                             [self.__org],
                                             time=PddlConditionEffectDto.AT_START)
        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(robot_at,
                                          [self.__dst],
                                          time=PddlConditionEffectDto.AT_END)

        effect_2 = PddlConditionEffectDto(robot_at,
                                          [self.__org],
                                          is_negative=True,
                                          time=PddlConditionEffectDto.AT_START)

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2NavigationAction()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
