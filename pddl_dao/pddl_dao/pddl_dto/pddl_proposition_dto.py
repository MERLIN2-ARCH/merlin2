
""" Pddl Proposition Dto """

from typing import List
from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto


class PddlPropositionDto(PddlDto):
    """ Pddl Proposition Dto Class """

    def __init__(self, PddlPredicateModel: PddlPredicateDto,
                 pddl_objects_list: List[PddlObjectDto] = None,
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

    def get_pddl_predicate(self) -> PddlPredicateDto:
        """ pddl predicate getter

        Returns:
            PddlPredicateDto: pddl predicate
        """

        return self._pddl_predicate

    def set_pddl_predicate(self, pddl_predicate: PddlPredicateDto):
        """ pddl predicate setter

        Args:
            pddl_predicate (PddlPredicateDto): pddl predicate
        """

        self._pddl_predicate = pddl_predicate

    def get_pddl_objects_list(self) -> List[PddlObjectDto]:
        """ pddl objects list getter

        Returns:
            List[PddlObjectDto]: list of pddl objects
        """

        return self._pddl_objects_list

    def set_pddl_objects_list(self, pddl_objects_list: List[PddlObjectDto]):
        """ pddl objects list setter

        Args:
            pddl_objects_list (List[PddlObjectDto]): list of pddl objects
        """

        if pddl_objects_list:
            self._pddl_objects_list = pddl_objects_list
        else:
            self._pddl_objects_list = []

    def __str__(self):
        string = "(" + self._pddl_predicate._predicate_name

        for pddl_object in self._pddl_objects_list:
            string += " " + pddl_object.get_object_name()

        string += ")"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlPropositionDto):

            if not other.get_pddl_predicate() == self.get_pddl_predicate():
                return False

            if not len(other.get_pddl_objects_list()) == len(self.get_pddl_objects_list()):
                return False

            for pddl_object, other_pddl_object in zip(self.get_pddl_objects_list(),
                                                      other.get_pddl_objects_list()):
                if not pddl_object == other_pddl_object:
                    return False

            return True

        return False
