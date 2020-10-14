
""" Pddl Dao Facory Interface """

from abc import ABC, abstractmethod
from pddl_dao.pddl_dao_interface.pddl_dao_type import PddlDaoType
from pddl_dao.pddl_dao_interface.pddl_dao_object import PddlDaoObject
from pddl_dao.pddl_dao_interface.pddl_dao_predicate import PddlDaoPredicate
from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PddlDaoProposition
from pddl_dao.pddl_dao_interface.pddl_dao_action import PddlDaoAction


class PddlDaoFactory(ABC):
    """ Pddl Dao Facory Abstract Class """

    @abstractmethod
    def create_pddl_dao_type(self) -> PddlDaoType:
        """ create a pddl dao type object

        Returns:
            PddlDaoType: dao for pddl type
        """

    @abstractmethod
    def create_pddl_dao_predicate(self) -> PddlDaoPredicate:
        """ create a pddl dao predicate object

        Returns:
            PddlDaoPredicate: dao for pddl predicate
        """

    @abstractmethod
    def create_pddl_dao_action(self) -> PddlDaoAction:
        """ create a pddl dao action object

        Returns:
            PddlDaoAction: dao for pddl action
        """

    @abstractmethod
    def create_pddl_dao_object(self) -> PddlDaoObject:
        """ create a pddl dao object object

        Returns:
            PddlDaoObject: dao for pddl object
        """

    @abstractmethod
    def create_pddl_dao_proposition(self) -> PddlDaoProposition:
        """ create a pddl dao type object

        Returns:
            PddlDaoProposition: dao for pddl proposition
        """
