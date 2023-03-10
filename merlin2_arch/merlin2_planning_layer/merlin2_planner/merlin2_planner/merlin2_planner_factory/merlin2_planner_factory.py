
""" Merlin2 Planner Factory """

from merlin2_planner.merlin2_planner_factory.merlin2_planners import Merlin2Planners
from merlin2_planner.merlin2_planners import (
    PopfMerlin2Planner,
    SmtpMerlin2Planner,
    UpMerlin2Planner,
    VhpopMerlin2Planner,
    Merlin2Planner
)


class Merlin2PlannerFactory:
    """ Merlin2 Planner Factory Class """

    def __init__(self):
        self.planners = Merlin2Planners
        self.__num_to_planner = {
            self.planners.POPF: PopfMerlin2Planner,
            self.planners.SMTP: SmtpMerlin2Planner,
            self.planners.UP: UpMerlin2Planner,
            self.planners.VHPOP: VhpopMerlin2Planner
        }

    def create_planner(self, planner_num: int) -> Merlin2Planner:
        """ create a planner

        Args:
            planner_num (int): number of the planner to create

        Returns:
            Merlin2Planner: planner
        """

        return self.__num_to_planner[planner_num]()
