
from pddl_dao.pddl_dto.pddl_dto import PDDL_DTO
from typing import List
from pddl_dao.pddl_dto.pddl_dto_proposition import PDDL_DTO_Proposition
from pddl_dao.pddl_dto.pddl_dto_predicate import PDDL_DTO_Predicate
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object


class PDDL_DTO_ConditionEffect(PDDL_DTO_Proposition, PDDL_DTO):

    AT_START = "at start"
    AT_END = "at end"
    OVER_ALL = "over all"

    def __init__(self, time: str,
                 pddl_predicate: PDDL_DTO_Predicate,
                 pddl_objects_list: List[PDDL_DTO_Object] = None,
                 is_negative: bool = False):

        self.set_time(time)
        self.set_is_negative(is_negative)

        PDDL_DTO_Proposition.__init__(
            self, pddl_predicate, pddl_objects_list)
        PDDL_DTO.__init__(self)

    def get_time(self) -> str:
        return self._time

    def set_time(self, time: str):
        self._time = time

    def get_is_negative(self) -> bool:
        return self._is_negative

    def set_is_negative(self, is_negative: bool):
        self._is_negative = is_negative

    def __str__(self):
        super_string = super(PDDL_DTO_ConditionEffect, self).__str__()
        string = "(" + self._time + " "
        if(self._is_negative):
            string += "(not "

        string += super_string

        string += ")"
        if(self._is_negative):
            string += ")"

        return string
