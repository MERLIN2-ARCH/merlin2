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

    def __init__(self) -> None:
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
