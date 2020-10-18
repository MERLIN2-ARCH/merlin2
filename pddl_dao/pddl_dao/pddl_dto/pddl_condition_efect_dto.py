
""" Pddl Condition/Effect Dto """

from typing import List
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto


class PddlConditionEffectDto(PddlPropositionDto):
    """ Pddl Condition/Effect Class Dto """

    AT_START = "at start"
    AT_END = "at end"
    OVER_ALL = "over all"

    def __init__(self,
                 pddl_predicate_dto: PddlPredicateDto,
                 pddl_objects_list: List[PddlObjectDto] = None,
                 time: str = None,
                 is_negative: bool = False):

        self.set_time(time)
        self.set_is_negative(is_negative)

        super().__init__(pddl_predicate_dto, pddl_objects_list)

    def get_time(self) -> str:
        """ time getter

        Returns:
            str: time the condition/effect will be resolved
        """

        return self._time

    def set_time(self, time: str):
        """ time setter

        Args:
            time (str): time the condition/effect will be resolved
        """

        self._time = time

    def get_is_negative(self) -> bool:
        """ is negative getter

        Returns:
            bool: is this condition/effect negative
        """

        return self._is_negative

    def set_is_negative(self, is_negative: bool):
        """ is negative setter

        Args:
            is_negative (bool): is this condition/effect negative
        """

        self._is_negative = is_negative

    def __str__(self):

        string = "(" + self._pddl_predicate._predicate_name

        for pddl_object in self._pddl_objects_list:
            string += " ?" + pddl_object.get_object_name()

        string += ")"

        if self._is_negative:
            string = "(not " + string + ")"

        if self._time:
            string = "(" + self._time + " " + string + ")"

        return string
