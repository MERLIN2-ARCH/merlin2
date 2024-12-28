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


""" Pddl Domain Generator """

from typing import List

from kant_dto import (
    PddlTypeDto,
    PddlPredicateDto,
    PddlActionDto
)


class Merlin2PddlDomainParser:
    """ Pddl Domain Generator Class """

    def __init__(self):
        self._requirements = "(:requirements :typing :negative-preconditions :durative-actions)"

    def parse_pddl_type_dto_list(self, pddl_type_dto_list: List[PddlTypeDto]) -> str:
        """ this method generates the string of the pddl types
            using a list of pddl type dtos

        Args:
            pddl_type_dto_list (List[PddlTypeDto]): list of pddl type dtos

        Returns:
            str: str of pddl types
        """

        string = "(:types\n"

        for pddl_type_dto in pddl_type_dto_list:
            string += "\t" + str(pddl_type_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_predicate_dto_list(self, pddl_predicate_dto_list: List[PddlPredicateDto]) -> str:
        """ this method generates the string of the pddl predicates
            using a list of pddl predicate dtos

        Args:
            pddl_predicate_dto_list (List[PddlPredicateDto]): list of pddl predicate dtos

        Returns:
            str: str of pddl predicates
        """

        string = "(:predicates\n"

        for pddl_predicate_dto in pddl_predicate_dto_list:
            string += "\t" + str(pddl_predicate_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_action_dto_list(self, pddl_action_dto_list: List[PddlActionDto]) -> str:
        """ this method generates the string of the pddl actions
            using a list of pddl action dtos

        Args:
            pddl_action_dto_list (List[PddlActionDto]): list of pddl action dtos

        Returns:
            str: str of pddl actions
        """

        string = ""

        for pddl_action_dto in pddl_action_dto_list:
            string += str(pddl_action_dto) + "\n"

        return string

    def parse_pddl_domain_dto(self,
                              pddl_type_dto_list: List[PddlTypeDto],
                              pddl_predicate_dto_list: List[PddlPredicateDto],
                              pddl_action_dto_list: List[PddlActionDto],
                              domain_name: str = "merlin2") -> str:
        """ this method generates the string pddl of the domain
            using lists of pddl types dtos, predicates dtos and action dtos

        Args:
            pddl_type_dto_list (List[PddlTypeDto]): list of pddl type dtos
            pddl_predicate_dto_list (List[PddlPredicateDto]): list of pddl predicate dtos
            pddl_action_dto_list (List[PddlActionDto]): list of pddl action dtos
            domain_name (str, optional): Pddl domain name. Defaults to "merlin2".

        Returns:
            str: str of the pddl domain
        """

        string = "(define (domain " + domain_name + ")\n"
        string += self._requirements + "\n"
        string += self.parse_pddl_type_dto_list(pddl_type_dto_list)
        string += self.parse_pddl_predicate_dto_list(pddl_predicate_dto_list)
        string += self.parse_pddl_action_dto_list(pddl_action_dto_list)
        string += ")\n"

        return string
