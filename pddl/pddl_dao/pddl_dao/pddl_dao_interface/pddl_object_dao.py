
""" Pddl Object Dao Interface """

from abc import abstractmethod
from pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dao_interface.pddl_dao import PddlDao


class PddlObjectDao(PddlDao):
    """ Pddl Object Dao Interface Abstract Class"""

    @abstractmethod
    def get(self, object_name: str) -> PddlObjectDto:
        """ get a PddlObjectDto with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlObjectDto: PddlObjectDto of the pddl object name
        """
