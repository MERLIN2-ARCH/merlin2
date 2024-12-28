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


""" Navigation State """

from rclpy.node import Node
from waypoint_navigation_msgs.action import NavigateToWp
from yasmin_ros import ActionState
from yasmin.blackboard import Blackboard


class Merlin2NavigationState(ActionState):
    """ Navigation State Class """

    def __init__(self, node: Node) -> None:

        super().__init__(
            NavigateToWp,
            "/waypoint_navigation/navigate_to_wp",
            self.create_nav_goal,
            node=node
        )

    def create_nav_goal(self, blackboard: Blackboard) -> NavigateToWp.Goal:
        """ create a goal for the waypoint navigation

            blackboard:
                destination

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            NavigateToWp.Goal: goal
        """

        goal = NavigateToWp.Goal()
        goal.wp_id = blackboard["destination"]
        return goal
