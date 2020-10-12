
from typing import List
from pddl_dao.pddl_dto.pddl_dto_predicate import PDDL_DTO_Predicate
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object


class PDDL_DTO_Proposition:

    def __init__(self, pddl_predicate: PDDL_DTO_Predicate,
                 pddl_objects_list: List[PDDL_DTO_Object] = None,
                 is_goal: bool = False):

        self.set_pddl_predicate(pddl_predicate)
        self.set_pddl_objects_list(pddl_objects_list)
        self.set_is_goal(is_goal)

    def get_is_goal(self) -> bool:
        return self._is_goal

    def set_is_goal(self, is_goal: bool):
        self._is_goal = is_goal

    def get_pddl_predicate(self) -> PDDL_DTO_Predicate:
        return self._pddl_predicate

    def set_pddl_predicate(self, pddl_predicate: PDDL_DTO_Predicate):
        self._pddl_predicate = pddl_predicate

    def get_pddl_objects_list(self) -> List[PDDL_DTO_Object]:
        return self._pddl_objects_list

    def set_pddl_objects_list(self, pddl_objects_list: List[PDDL_DTO_Object]):
        if(pddl_objects_list):
            self._pddl_objects_list = pddl_objects_list
        else:
            self._pddl_objects_list = []

    def __str__(self):
        string = "(" + self._pddl_predicate._predicate_name

        for pddl_object in self._pddl_objects_list:
            string += " " + pddl_object.get_object_name()

        string += ")"

        return string
