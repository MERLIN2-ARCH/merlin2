
""" Pddl Dto Object """

from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class PddlDtoObject(PddlDto):
    """ Pddl Dto Object Class """

    def __init__(self, pddl_type: PddlDtoType, object_name: str):

        self.set_pddl_type(pddl_type)
        self.set_object_name(object_name)

        PddlDto.__init__(self)

    def get_pddl_type(self) -> PddlDtoType:
        """ pddl type getter

        Returns:
            PddlDtoType: pddl type
        """

        return self._pddl_type

    def set_pddl_type(self, pddl_type: PddlDtoType):
        """ pddl type setter

        Args:
            pddl_type (PddlDtoType): pddl type
        """

        self._pddl_type = pddl_type

    def get_object_name(self) -> str:
        """ pddl object name getter

        Returns:
            str: pddl object name
        """

        return self._object_name

    def set_object_name(self, object_name: str):
        """ pddl object name setter

        Args:
            object_name (str): pddl object name
        """

        self._object_name = object_name

    def __str__(self):
        return self._object_name + " - " + self._pddl_type.get_type_name()
