from abc import ABC, abstractmethod
from pddl_dao.pddl_dao_interface.pddl_dao_type import PddlDaoType
from pddl_dao.pddl_dao_interface.pddl_dao_object import PddlDaoObject
from pddl_dao.pddl_dao_interface.pddl_dao_predicate import PddlDaoPredicate
from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PddlDaoProposition
from pddl_dao.pddl_dao_interface.pddl_dao_action import PddlDaoAction


class PDDL_DAO_Factory(ABC):

    @abstractmethod
    def create_dao_pddl_type(self) -> PddlDaoType:
        pass

    @abstractmethod
    def create_dao_pddl_predicate(self) -> PddlDaoPredicate:
        pass

    @abstractmethod
    def create_dao_pddl_action(self) -> PddlDaoAction:
        pass

    @abstractmethod
    def create_dao_pddl_object(self) -> PddlDaoObject:
        pass

    @abstractmethod
    def create_dao_pddl_proposition(self) -> PddlDaoProposition:
        pass
