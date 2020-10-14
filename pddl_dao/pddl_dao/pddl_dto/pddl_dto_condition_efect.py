
""" Pddl Dto Condition/Effect """

from typing import List
from pddl_dao.pddl_dto.pddl_dto_proposition import PddlDtoProposition
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class PddlDtoConditionEffect(PddlDtoProposition):
    """ Pddl Dto Condition/Effect Class """

    AT_START = "at start"
    AT_END = "at end"
    OVER_ALL = "over all"

    def __init__(self, time: str,
                 pddl_predicate: PddlDtoPredicate,
                 pddl_objects_list: List[PddlDtoObject] = None,
                 is_negative: bool = False):

        self.set_time(time)
        self.set_is_negative(is_negative)

        PddlDtoProposition.__init__(
            self, pddl_predicate, pddl_objects_list)

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
        super_string = super(PddlDtoConditionEffect, self).__str__()

        string = "(" + self._time + " "
        if self._is_negative:
            string += "(not "

        string += super_string

        string += ")"
        if self._is_negative:
            string += ")"

        return string
