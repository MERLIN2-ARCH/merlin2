
""" Merlin2 Planner """

from abc import ABC, abstractmethod


class Merlin2Planner(ABC):
    """ Merlin2 Planner Abstract Class """

    @abstractmethod
    def plan(self, domain: str, problem: str) -> str:
        """ create a ppdl plan

        Args:
            domain (str): str of a pddl domain
            problem (str): str of a pddl problem

        Returns:
            str: pddl of the plan
        """
