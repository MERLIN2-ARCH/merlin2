
""" Merlin2 Planner """

from typing import List
from abc import ABC, abstractmethod
from merlin2_msgs.msg import PlanAction


class Merlin2Planner(ABC):
    """ Merlin2 Planner Abstract Class """

    def __init__(self):
        self._has_solution = False
        self._str_plan = ""
        self._plan_actions = []

    @abstractmethod
    def _generate_plan(self, domain: str, problem: str):
        """ create a ppdl plan

        Args:
            domain (str): str of a pddl domain
            problem (str): str of a pddl problem
        """

    @abstractmethod
    def _parse_plan(self):
        """ parse the current plan from str to
            list of PlanAction and check if has solution
        """

    def generate_plan(self, domain: str, problem: str):
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
