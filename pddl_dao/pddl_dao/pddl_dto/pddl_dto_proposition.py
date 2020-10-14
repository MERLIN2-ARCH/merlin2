
""" Pddl Dto Proposition """

from typing import List
from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class PddlDtoProposition(PddlDto):
    """ Pddl Dto Proposition Class """

    def __init__(self, PddlPredicateModel: PddlDtoPredicate,
                 pddl_objects_list: List[PddlDtoObject] = None,
                 is_goal: bool = False):

        self.set_pddl_predicate(PddlPredicateModel)
        self.set_pddl_objects_list(pddl_objects_list)
        self.set_is_goal(is_goal)

        PddlDto.__init__(self)

    def get_is_goal(self) -> bool:
        """ is goal getter

        Returns:
            bool: is this proposition a goal
        """

        return self._is_goal

    def set_is_goal(self, is_goal: bool):
        """ is goal setter

        Args:
            is_goal (bool): is this proposition a goal
        """

        self._is_goal = is_goal

    def get_pddl_predicate(self) -> PddlDtoPredicate:
        """ pddl predicate getter

        Returns:
            PddlDtoPredicate: pddl predicate
        """

        return self._pddl_predicate

    def set_pddl_predicate(self, PddlPredicateModel: PddlDtoPredicate):
        """ pddl predicate setter

        Args:
            PddlPredicateModel (PddlDtoPredicate): pddl predicate
        """

        self._pddl_predicate = PddlPredicateModel

    def get_pddl_objects_list(self) -> List[PddlDtoObject]:
        """ pddl objects list getter

        Returns:
            List[PddlDtoObject]: list of pddl objects
        """

        return self._pddl_objects_list

    def set_pddl_objects_list(self, pddl_objects_list: List[PddlDtoObject]):
        """ pddl objects list setter

        Args:
            pddl_objects_list (List[PddlDtoObject]): list of pddl objects
        """

        if pddl_objects_list:
            self._pddl_objects_list = pddl_objects_list
        else:
            self._pddl_objects_list = []

    def __str__(self):
        string = "(" + self._pddl_predicate._predicate_name

        for PddlObjectModel in self._pddl_objects_list:
            string += " " + PddlObjectModel.get_object_name()

        string += ")"

        return string
