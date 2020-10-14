
""" Pddl Dao Type Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class PddlDaoType(ABC):
    """ Pddl Dao Type Abstract Class """

    @abstractmethod
    def get(self, type_name: str) -> PddlDtoType:
        """ get a PddlDtoType with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlDtoType: PddlDtoType of the pddl type name
        """

    @abstractmethod
    def get_all(self) -> List[PddlDtoType]:
        """ get all PddlDtoType

        Returns:
            List[PddlDtoType]: list of all PddlDtoType
        """

    @abstractmethod
    def _save(self, pddl_dto_type: PddlDtoType) -> bool:
        """ save a PddlDtoType
            if the PddlDtoType is already saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pddl_dto_type: PddlDtoType) -> bool:
        """ update a PddlDtoType
            if the PddlDtoType is not saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pddl_dto_type: PddlDtoType) -> bool:
        """ save or update a PddlDtoType
            if the PddlDtoType is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pddl_dto_type: PddlDtoType) -> bool:
        """ delete a PddlDtoType
            if the PddlDtoType is not saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl types

        Returns:
            bool: succeed
        """
