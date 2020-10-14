
""" Pddl Object Dto """

from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto


class PddlObjectDto(PddlDto):
    """ Pddl Object Dto Class """

    def __init__(self, PddlTypeModel: PddlTypeDto, object_name: str):

        self.set_pddl_type(PddlTypeModel)
        self.set_object_name(object_name)

        PddlDto.__init__(self)

    def get_pddl_type(self) -> PddlTypeDto:
        """ pddl type getter

        Returns:
            PddlTypeDto: pddl type
        """

        return self._pddl_type

    def set_pddl_type(self, PddlTypeModel: PddlTypeDto):
        """ pddl type setter

        Args:
            PddlTypeModel (PddlTypeDto): pddl type
        """

        self._pddl_type = PddlTypeModel

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
