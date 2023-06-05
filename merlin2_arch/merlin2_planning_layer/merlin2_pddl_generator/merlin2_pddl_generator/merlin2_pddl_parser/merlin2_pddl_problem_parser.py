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


""" Pddl Problem Generator """

from typing import List
from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)


class Merlin2PddlProblemParser:
    """ Pddl Problem Generator Class """

    def parse_pddl_object_dto_list(self, pddl_object_dto_list: List[PddlObjectDto]) -> str:
        """ this method generates the string of the pddl objects
            using a list of pddl object dtos

        Args:
            pddl_object_dto_list (List[PddlObjectDto]): list of pddl object dtos

        Returns:
            str: str of pddl objects
        """

        string = "(:objects\n"

        for pddl_object_dto in pddl_object_dto_list:
            string += "\t" + str(pddl_object_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_proposition_dto_list(self, pddl_proposition_dto_list: List[PddlPropositionDto]) -> str:
        """ this method generates the string of the pddl propositions
            using a list of pddl proposition dtos

        Args:
            pddl_proposition_dto_list (List[PddlPropositionDto]): list of pddl object dtos

        Returns:
            str: str of pddl propositions
        """

        string = "(:init\n"

        for pddl_proposition_dto in pddl_proposition_dto_list:
            string += "\t" + str(pddl_proposition_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_goal_dto_list(self, pddl_dto_goals_list: List[PddlPropositionDto]) -> str:
        """ this method generates the string of the pddl goals
            using a list of pddl goal dtos

        Args:
            pddl_dto_goals_list (List[PddlPropositionDto]): list of pddl goal dtos

        Returns:
            str: str of pddl goals
        """

        string = "(:goal\n"

        if len(pddl_dto_goals_list) > 1:
            string += "\t(and\n"

        for pddl_dto_goal in pddl_dto_goals_list:
            string += "\t" + str(pddl_dto_goal) + "\n"

        if len(pddl_dto_goals_list) > 1:
            string += "\t)\n"

        if len(pddl_dto_goals_list) == 0:
            string += "()"

        string += ")\n"

        return string

    def parse_pddl_problem_dto_list(self, pddl_object_dto_list: List[PddlObjectDto],
                                    pddl_proposition_dto_list: List[PddlPropositionDto],
                                    pddl_dto_goals_list: List[PddlPropositionDto],
                                    domain_name: str = "merlin2",
                                    problem_name: str = "merlin2_prb") -> str:
        """ this method generates the string pddl of the problem
            using lists of pddl objects dtos, propositions dtos and goals dtos

        Args:
            pddl_object_dto (List[PddlObjectDto]): list of pddl object dtos
            pddl_proposition_dto (List[PddlPropositionDto]): list of pddl proposition dtos
            pddl_dto_goals (List[PddlPropositionDto]): list of pddl goal dtos
            domain_name (str, optional): Pddl domain name. Defaults to "merlin2".
            problem_name (str, optional): Problem domain name. Defaults to "merlin2_prb".

        Returns:
            str: [description]
        """

        string = "(define (problem " + problem_name + \
            ")\n(:domain " + domain_name + ")\n"
        string += self.parse_pddl_object_dto_list(pddl_object_dto_list)
        string += self.parse_pddl_proposition_dto_list(
            pddl_proposition_dto_list)
        string += self.parse_pddl_goal_dto_list(pddl_dto_goals_list)
        string += ")\n"

        return string
