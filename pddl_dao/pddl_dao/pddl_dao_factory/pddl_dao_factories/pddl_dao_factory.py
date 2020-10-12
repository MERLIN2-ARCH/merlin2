from abc import ABC, abstractmethod
from pddl_dao.pddl_dao_interface.pddl_dao_type import PDDL_DAO_Type
from pddl_dao.pddl_dao_interface.pddl_dao_object import PDDL_DAO_Object
from pddl_dao.pddl_dao_interface.pddl_dao_predicate import PDDL_DAO_Predicate
from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PDDL_DAO_Proposition
from pddl_dao.pddl_dao_interface.pddl_dao_action import PDDL_DAO_Action


class PDDL_DAO_Factory(ABC):

    @abstractmethod
    def create_dao_pddl_type(self) -> PDDL_DAO_Type:
        pass

    @abstractmethod
    def create_dao_pddl_predicate(self) -> PDDL_DAO_Predicate:
        pass

    @abstractmethod
    def create_dao_pddl_action(self) -> PDDL_DAO_Action:
        pass

    @abstractmethod
    def create_dao_pddl_object(self) -> PDDL_DAO_Object:
        pass

    @abstractmethod
    def create_dao_pddl_proposition(self) -> PDDL_DAO_Proposition:
        pass
