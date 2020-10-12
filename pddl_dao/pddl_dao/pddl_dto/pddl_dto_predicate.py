
from pddl_dao.pddl_dto.pddl_dto import PDDL_DTO
from typing import List
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type


class PDDL_DTO_Predicate:

    def __init__(self, predicate_name: str, pddl_types_list: List[PDDL_DTO_Type] = None):

        self.set_predicate_name(predicate_name)
        self.set_pddl_types_list(pddl_types_list)

        PDDL_DTO.__init__(self)

    def get_predicate_name(self) -> str:
        return self._predicate_name

    def set_predicate_name(self, predicate_name: str):
        self._predicate_name = predicate_name

    def get_pddl_types_list(self) -> List[PDDL_DTO_Type]:
        return self._pddl_types_list

    def set_pddl_types_list(self, pddl_types_list: List[PDDL_DTO_Type]):
        if(pddl_types_list):
            self._pddl_types_list = pddl_types_list
        else:
            self._pddl_types_list = []

    def __str__(self):
        string = "(" + self._predicate_name

        if(self._pddl_types_list):
            for i in range(len(self._pddl_types_list)):
                pddl_type = self._pddl_types_list[i]
                type_name = pddl_type.get_type_name()
                string += " ?" + type_name[0] + str(i) + " - " + type_name

        string += ")"

        return string
