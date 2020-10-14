
""" Pddl Dto Type """

from pddl_dao.pddl_dto.pddl_dto import PddlDto


class PddlDtoType(PddlDto):
    """ Pddl Dto Type Class """

    def __init__(self, type_name: str):

        self.set_type_name(type_name)

        PddlDto.__init__(self)

    def get_type_name(self) -> str:
        """ pddl type name getter

        Returns:
            str: pddl type name
        """

        return self._type_name

    def set_type_name(self, type_name: str):
        """ pddl type name setter

        Args:
            type_name (str): pddl type name
        """

        self._type_name = type_name

    def __str__(self):
        return self._type_name
