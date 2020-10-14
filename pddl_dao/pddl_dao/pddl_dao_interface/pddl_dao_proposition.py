
""" Pddl Dao Proposition Interface """

from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_proposition import PddlDtoProposition


class PddlDaoProposition(ABC):
    """ Pddl Dao Proposition Abstract Class """

    @abstractmethod
    def get_by_predicate(self, predicate_name: str) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition with a given pddl predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

    @abstractmethod
    def get_goals(self) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition that are goals

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

    @abstractmethod
    def get_all(self) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

    @abstractmethod
    def _save(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ save a PddlDtoProposition
            if the PddlDtoProposition is already saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ update a PddlDtoProposition
            if the PddlDtoProposition is not saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ save or update a PddlDtoProposition
            if the PddlDtoProposition is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ delete a PddlDtoProposition
            if the PddlDtoProposition is not saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl propositions

        Returns:
            bool: succeed
        """
