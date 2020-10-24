
""" Pddl Predicate Dto """

from typing import List
from pddl_dto.pddl_dto import PddlDto
from pddl_dto.pddl_type_dto import PddlTypeDto


class PddlPredicateDto(PddlDto):
    """ Pddl Predicate Dto Class """

    def __init__(self, predicate_name: str, pddl_types_list: List[PddlTypeDto] = None):

        self.set_predicate_name(predicate_name)
        self.set_pddl_types_list(pddl_types_list)

        PddlDto.__init__(self)

    def get_predicate_name(self) -> str:
        """ pddl predicate name getter

        Returns:
            str: predicate name
        """

        return self._predicate_name

    def set_predicate_name(self, predicate_name: str):
        """ pddl predicate name setter

        Args:
            predicate_name (str): pddl predicate name
        """

        self._predicate_name = predicate_name

    def get_pddl_types_list(self) -> List[PddlTypeDto]:
        """ pddl types list getter

        Returns:
            List[PddlTypeDto]: list of pddl types
        """

        return self._pddl_types_list

    def set_pddl_types_list(self, pddl_types_list: List[PddlTypeDto]):
        """ pddl types list setter

        Args:
            pddl_types_list (List[PddlTypeDto]): list of pddl types
        """

        if pddl_types_list:
            self._pddl_types_list = pddl_types_list
        else:
            self._pddl_types_list = []

    def __str__(self):
        string = "(" + self._predicate_name

        if self._pddl_types_list:
            for i in range(len(self._pddl_types_list)):
                pddl_type = self._pddl_types_list[i]
                type_name = pddl_type.get_type_name()
                string += " ?" + type_name[0] + str(i) + " - " + type_name

        string += ")"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlPredicateDto):
            return self.get_predicate_name() == other.get_predicate_name()

        return False
