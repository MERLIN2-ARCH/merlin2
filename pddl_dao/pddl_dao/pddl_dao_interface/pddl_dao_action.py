
""" Pddl Dao Action Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_action import PddlDtoAction


class PddlDaoAction(ABC):
    """ Pddl Dao Action Abstract Class """

    @abstractmethod
    def get(self, action_name: str) -> PddlDtoAction:
        """ get a PddlDtoAction with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlDtoAction: PddlDtoAction of the pddl action name
        """

    @abstractmethod
    def get_all(self) -> List[PddlDtoAction]:
        """ get all PddlDtoAction

        Returns:
            List[PddlDtoAction]: list of all PddlDtoAction
        """

    @abstractmethod
    def _save(self, pddl_dto_action: PddlDtoAction) -> bool:
        """ save a PddlDtoAction
            if the PddlDtoAction is already saved return False, else return True

        Args:
            pddl_dto_action (PddlDtoAction): PddlDtoAction to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pddl_dto_action: PddlDtoAction) -> bool:
        """ update a PddlDtoAction
            if the PddlDtoAction is not saved return False, else return True

        Args:
            pddl_dto_action (PddlDtoAction): PddlDtoAction to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pddl_dto_action: PddlDtoAction) -> bool:
        """ save or update a PddlDtoAction
            if the PddlDtoAction is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_action (PddlDtoAction): PddlDtoAction to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pddl_dto_action: PddlDtoAction) -> bool:
        """ delete a PddlDtoAction
            if the PddlDtoAction is not saved return False, else return True

        Args:
            pddl_dto_action (PddlDtoAction): PddlDtoAction to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl actions

        Returns:
            bool: succeed
        """
