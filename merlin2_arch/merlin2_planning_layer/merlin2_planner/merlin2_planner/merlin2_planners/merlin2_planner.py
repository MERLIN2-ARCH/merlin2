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


""" Merlin2 Planner """

from typing import List
from abc import ABC, abstractmethod
from merlin2_msgs.msg import PlanAction


class Merlin2Planner(ABC):
    """ Merlin2 Planner Abstract Class """

    def __init__(self) -> None:
        self._has_solution = False
        self._str_plan = ""
        self._plan_actions = []

    @abstractmethod
    def _generate_plan(self, domain: str, problem: str) -> None:
        """ create a ppdl plan

        Args:
            domain (str): str of a pddl domain
            problem (str): str of a pddl problem
        """

    @abstractmethod
    def _parse_plan(self) -> None:
        """ parse the current plan from str to
            list of PlanAction and check if has solution
        """

    def generate_plan(self, domain: str, problem: str) -> None:
        """ create and parse a pddl plan

        Args:
            domain (str): [description]
            problem (str): [description]
        """

        self._has_solution = False
        self._str_plan = ""
        self._plan_actions = []

        self._generate_plan(domain, problem)

        if self._str_plan:
            self._parse_plan()

    def has_solution(self) -> bool:
        """ returns if if generated plan has solution

        Returns:
            bool: does the plan has solution?
        """

        return self._has_solution

    def get_plan_actions(self) -> List[PlanAction]:
        """ get the PlanAction that composed the plan

        Returns:
            List[PlanAction]: list of PlanAction
        """

        return self._plan_actions

    def get_str_plan(self) -> str:
        """ get the str plan

        Returns:
            str: plan
        """

        return self._str_plan
