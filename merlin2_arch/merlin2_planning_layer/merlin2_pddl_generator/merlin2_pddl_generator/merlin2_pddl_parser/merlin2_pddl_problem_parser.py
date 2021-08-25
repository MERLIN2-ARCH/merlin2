
""" Pddl Problem Generator """

from typing import List
from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)


class Merlin2PddlProblemParser:
    """ Pddl Problem Generator Class """

    def parse_pddl_dto_objects(self, pddl_dto_objects: List[PddlObjectDto]) -> str:
        """ this method generates the string of the pddl objects
            using a list of pddl object dtos

        Args:
            pddl_dto_objects (List[PddlObjectDto]): list of pddl object dtos

        Returns:
            str: str of pddl objects
        """

        string = "(:objects\n"

        for pddl_dto_object in pddl_dto_objects:
            string += "\t" + str(pddl_dto_object) + "\n"

        string += ")\n"

        return string

    def parse_pddl_dto_propositions(self, pddl_dto_propositions: List[PddlPropositionDto]) -> str:
        """ this method generates the string of the pddl propositions
            using a list of pddl proposition dtos

        Args:
            pddl_dto_propositions (List[PddlPropositionDto]): list of pddl object dtos

        Returns:
            str: str of pddl propositions
        """

        string = "(:init\n"

        for pddl_dto_proposition in pddl_dto_propositions:
            string += "\t" + str(pddl_dto_proposition) + "\n"

        string += ")\n"

        return string

    def parse_pddl_dto_goals(self, pddl_dto_goals: List[PddlPropositionDto]) -> str:
        """ this method generates the string of the pddl goals
            using a list of pddl goal dtos

        Args:
            pddl_dto_goals (List[PddlPropositionDto]): list of pddl goal dtos

        Returns:
            str: str of pddl goals
        """

        string = "(:goal\n"

        if len(pddl_dto_goals) > 1:
            string += "\t(and\n"

        for pddl_dto_goal in pddl_dto_goals:
            string += "\t" + str(pddl_dto_goal) + "\n"

        if len(pddl_dto_goals) > 1:
            string += "\t)\n"

        if len(pddl_dto_goals) == 0:
            string += "()"

        string += ")\n"

        return string

    def parse_pddl_problem_dto(self, pddl_dto_objects: List[PddlObjectDto],
                               pddl_dto_propositions: List[PddlPropositionDto],
                               pddl_dto_goals: List[PddlPropositionDto],
                               domain_name: str = "merlin2",
                               problem_name: str = "merlin2_prb") -> str:
        """ this method generates the string pddl of the problem
            using lists of pddl objects dtos, propositions dtos and goals dtos

        Args:
            pddl_dto_objects (List[PddlObjectDto]): list of pddl object dtos
            pddl_dto_propositions (List[PddlPropositionDto]): list of pddl proposition dtos
            pddl_dto_goals (List[PddlPropositionDto]): list of pddl goal dtos
            domain_name (str, optional): Pddl domain name. Defaults to "merlin2".
            problem_name (str, optional): Problem domain name. Defaults to "merlin2_prb".

        Returns:
            str: [description]
        """

        string = "(define (problem " + problem_name + \
            ")\n(:domain " + domain_name + ")\n"
        string += self.parse_pddl_dto_objects(pddl_dto_objects)
        string += self.parse_pddl_dto_propositions(pddl_dto_propositions)
        string += self.parse_pddl_dto_goals(pddl_dto_goals)
        string += ")\n"

        return string
