
""" Navigation State """

from waypoint_navigation_interfaces.action import NavigateToWp
from yasmin_ros import AcionState
from yasmin.blackboard import Blackboard


class Merlin2NavigationState(AcionState):
    """ Navigation State Class """

    def __init__(self, node):

        super().__init__(node, NavigateToWp,
                         "/waypoint_navigation/navigate_to_wp", self.create_nav_goal)

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
        goal.wp_id = blackboard.destination
        return goal
