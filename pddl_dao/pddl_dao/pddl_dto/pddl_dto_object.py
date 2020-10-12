
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type


class PDDL_DTO_Object:

    def __init__(self, pddl_type: PDDL_DTO_Type, object_name: str):

        self.set_pddl_type(pddl_type)
        self.set_object_name(object_name)

    def get_pddl_type(self) -> PDDL_DTO_Type:
        return self._pddl_type

    def set_pddl_type(self, pddl_type: PDDL_DTO_Type):
        self._pddl_type = pddl_type

    def get_object_name(self) -> str:
        return self._object_name

    def set_object_name(self, object_name: str):
        self._object_name = object_name

    def __str__(self):
        return self._object_name + " - " + self._pddl_type.get_type_name()
