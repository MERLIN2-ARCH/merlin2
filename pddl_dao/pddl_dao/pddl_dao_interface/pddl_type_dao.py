
""" Pddl Type Dao Interface """

from abc import abstractmethod
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dao_interface.generic_pddl_dao import GenericPddlDao


class PddlTypeDao(GenericPddlDao):
    """ Pddl Type Dao Abstract Class """

    @abstractmethod
    def get(self, type_name: str) -> PddlTypeDto:
        """ get a PddlTypeDto with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlTypeDto: PddlTypeDto of the pddl type name
        """