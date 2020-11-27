
""" Pddl Dao Facory Interface """

from abc import ABC, abstractmethod
from pddl_dao.pddl_dao_interface import (
    PddlTypeDao,
    PddlObjectDao,
    PddlPredicateDao,
    PddlPropositionDao,
    PddlActionDao
)


class PddlDaoFactory(ABC):
    """ Pddl Dao Facory Abstract Class """

    @abstractmethod
    def create_pddl_type_dao(self) -> PddlTypeDao:
        """ create a pddl dao type object

        Returns:
            PddlTypeDao: dao for pddl type
        """

    @abstractmethod
    def create_pddl_predicate_dao(self) -> PddlPredicateDao:
        """ create a pddl dao predicate object

        Returns:
            PddlPredicateDao: dao for pddl predicate
        """

    @abstractmethod
    def create_pddl_action_dao(self) -> PddlActionDao:
        """ create a pddl dao action object

        Returns:
            PddlActionDao: dao for pddl action
        """

    @abstractmethod
    def create_pddl_object_dao(self) -> PddlObjectDao:
        """ create a pddl dao object object

        Returns:
            PddlObjectDao: dao for pddl object
        """

    @abstractmethod
    def create_pddl_proposition_dao(self) -> PddlPropositionDao:
        """ create a pddl dao type object

        Returns:
            PddlPropositionDao: dao for pddl proposition
        """
