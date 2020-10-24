
""" Pddl Domain Generator """

from typing import List
from pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dto.pddl_action_dto import PddlActionDto


class Merlin2PddlDomainParser:
    """ Pddl Domain Generator Class """

    def __init__(self):
        self._requirements = "(:requirements :typing :negative-preconditions :durative-actions)"

    def parse_pddl_type_dtos(self, pddl_type_dtos: List[PddlTypeDto]) -> str:
        """ this method generates the string of the pddl types
            using a list of pddl type dtos

        Args:
            pddl_type_dtos (List[PddlTypeDto]): list of pddl type dtos

        Returns:
            str: str of pddl types
        """

        string = "(:types\n"

        for pddl_type_dto in pddl_type_dtos:
            string += "\t" + str(pddl_type_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_predicate_dtos(self, pddl_predicate_dtos: List[PddlPredicateDto]) -> str:
        """ this method generates the string of the pddl predicates
            using a list of pddl predicate dtos

        Args:
            pddl_predicate_dtos (List[PddlPredicateDto]): list of pddl predicate dtos

        Returns:
            str: str of pddl predicates
        """

        string = "(:predicates\n"

        for pddl_predicate_dto in pddl_predicate_dtos:
            string += "\t" + str(pddl_predicate_dto) + "\n"

        string += ")\n"

        return string

    def parse_pddl_action_dtos(self, pddl_action_dtos: List[PddlActionDto]) -> str:
        """ this method generates the string of the pddl actions
            using a list of pddl action dtos

        Args:
            pddl_action_dtos (List[PddlActionDto]): list of pddl action dtos

        Returns:
            str: str of pddl actions
        """

        string = ""

        for pddl_action_dto in pddl_action_dtos:
            string += str(pddl_action_dto) + "\n"

        return string

    def parse_pddl_domain_dto(self,
                              pddl_type_dtos: List[PddlTypeDto],
                              pddl_predicate_dtos: List[PddlPredicateDto],
                              pddl_action_dtos: List[PddlActionDto],
                              domain_name: str = "merlin2") -> str:
        """ this method generates the string pddl of the domain
            using lists of pddl types dtos, predicates dtos and action dtos

        Args:
            pddl_type_dtos (List[PddlTypeDto]): list of pddl type dtos
            pddl_predicate_dtos (List[PddlPredicateDto]): list of pddl predicate dtos
            pddl_action_dtos (List[PddlActionDto]): list of pddl action dtos
            domain_name (str, optional): Pddl domain name. Defaults to "merlin2".

        Returns:
            str: str of the pddl domain
        """

        string = "(define (domain " + domain_name + ")\n"
        string += self._requirements + "\n"
        string += self.parse_pddl_type_dtos(pddl_type_dtos)
        string += self.parse_pddl_predicate_dtos(pddl_predicate_dtos)
        string += self.parse_pddl_action_dtos(pddl_action_dtos)
        string += ")\n"

        return string
