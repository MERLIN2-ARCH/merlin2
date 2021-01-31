
""" Navigation State """

from ros2_topological_nav_interfaces.action import TopoNav
from ros2_fsm.ros2_states import AcionState
from ros2_fsm.basic_fsm.blackboard import Blackboard


class Merlin2NavigationState(AcionState):
    """ Navigation State Class """

    def __init__(self, node):

        super().__init__(node, TopoNav, "/topo_nav/navigation", self.create_nav_goal)

    def create_nav_goal(self, blackboard: Blackboard) -> TopoNav.Goal:
        """ create a goal for the topological navigation

            blackboard:
                destination

        Args:
            blackboard (Blackboard): blackboard of the fsm

        Returns:
            TopoNav.Goal: goal
        """

        goal = TopoNav.Goal()
        goal.point = blackboard.destination
        return goal
