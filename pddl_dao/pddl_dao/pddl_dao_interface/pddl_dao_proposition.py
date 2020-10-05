
from abc import ABC, abstractmethod


class PDDL_DAO_Proposition(ABC):

    @abstractmethod
    def get_by_predicate(self):
        pass

    @abstractmethod
    def get_goals(self):
        pass

    @abstractmethod
    def get_all(self):
        pass

    @abstractmethod
    def save(self, pddl_dto_proposition):
        pass

    @abstractmethod
    def update(self, pddl_dto_proposition):
        pass

    @abstractmethod
    def save_update(self, pddl_dto_proposition):
        pass

    @abstractmethod
    def delete(self, pddl_dto_proposition):
        pass

    @abstractmethod
    def delete_all(self):
        pass
