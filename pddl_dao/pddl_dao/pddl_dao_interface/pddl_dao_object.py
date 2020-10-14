
""" Pddl Dao Object Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class PddlDaoObject(ABC):
    """ Pddl Dao Object Interface Abstract Class"""

    @abstractmethod
    def get(self, object_name: str) -> PddlDtoObject:
        """ get a PddlDtoObject with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlDtoObject: PddlDtoObject of the pddl object name
        """

    @abstractmethod
    def get_all(self) -> List[PddlDtoObject]:
        """ get all PddlDtoObject

        Returns:
            List[PddlDtoObject]: list of all PddlDtoObject
        """

    @abstractmethod
    def _save(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ save a PddlDtoObject
            if the PddlDtoObject is already saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ update a PddlDtoObject
            if the PddlDtoObject is not saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ save or update a PddlDtoObject
            if the PddlDtoObject is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ delete a PddlDtoObject
            if the PddlDtoObject is not saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl objects

        Returns:
            bool: succeed
        """
