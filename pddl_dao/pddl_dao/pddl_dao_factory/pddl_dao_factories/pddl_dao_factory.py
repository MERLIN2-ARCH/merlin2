from abc import ABC, abstractmethod


class PDDL_DAO_Factory(ABC):

    @abstractmethod
    def create_dao_pddl_type(self):
        pass

    @abstractmethod
    def create_dao_pddl_predicate(self):
        pass

    @abstractmethod
    def create_dao_pddl_action(self):
        pass

    @abstractmethod
    def create_dao_pddl_object(self):
        pass

    @abstractmethod
    def create_dao_pddl_proposition(self):
        pass
