
""" Generic Pddl Dao Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto import PddlDto


class GenericPddlDao(ABC):
    """ Generic Pddl Dao Abstract Class """

    @abstractmethod
    def get_all(self) -> List[PddlDto]:
        """ get all PddlDto

        Returns:
            List[PddlDto]: list of all PddlDto
        """

    @abstractmethod
    def _save(self, pdd_dto: PddlDto) -> bool:
        """ save a PddlDto
            if the PddlDto is already saved return False, else return True

        Args:
            pdd_dto (PddlDto): PddlDto to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pdd_dto: PddlDto) -> bool:
        """ update a PddlDto
            if the PddlDto is not saved return False, else return True

        Args:
            pdd_dto (PddlDto): PddlDto to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pdd_dto: PddlDto) -> bool:
        """ save or update a PddlDto
            if the PddlDto is not saved it will be saved, else it will be updated

        Args:
            pdd_dto (PddlDto): PddlDto to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pdd_dto: PddlDto) -> bool:
        """ delete a PddlDto
            if the PddlDto is not saved return False, else return True

        Args:
            pdd_dto (PddlDto): PddlDto to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl dtos

        Returns:
            bool: succeed
        """
