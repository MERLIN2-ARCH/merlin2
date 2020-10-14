
""" Pddl Dao Predicate Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate


class PddlDaoPredicate(ABC):
    """ Pddl Dao Predicate Abstract Class """

    @abstractmethod
    def get(self, predicate_name: str) -> PddlDtoPredicate:
        """ get a PddlDtoPredicate with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlDtoPredicate: PddlDtoPredicate of the pddl predicate name
        """

    @abstractmethod
    def get_all(self) -> List[PddlDtoPredicate]:
        """ get all PddlDtoPredicate

        Returns:
            List[PddlDtoPredicate]: list of all PddlDtoPredicate
        """

    @abstractmethod
    def _save(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        """ save a PddlDtoPredicate
            if the PddlDtoPredicate is already saved return False, else return True

        Args:
            pddl_dto_predicate (PddlDtoPredicate): PddlDtoPredicate to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        """ update a PddlDtoPredicate
             if the PddlDtoPredicate is not saved return False, else return True

         Args:
             pddl_dto_predicate (PddlDtoPredicate): PddlDtoPredicate to update

         Returns:
             bool: succeed
         """

    @abstractmethod
    def save(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        """ save or update a PddlDtoPredicate
            if the PddlDtoPredicate is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_predicate (PddlDtoPredicate): PddlDtoPredicate to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        """ delete a PddlDtoPredicate
            if the PddlDtoPredicate is not saved return False, else return True

        Args:
            pddl_dto_predicate (PddlDtoPredicate): PddlDtoPredicate to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl predicates

        Returns:
            bool: succeed
        """
