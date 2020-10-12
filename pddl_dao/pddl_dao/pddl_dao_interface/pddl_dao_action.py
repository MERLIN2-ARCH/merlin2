
from abc import ABC, abstractmethod


class PDDL_DAO_Action(ABC):

    @abstractmethod
    def get(self, action_name):
        pass

    @abstractmethod
    def get_all(self):
        pass

    @abstractmethod
    def _save(self, pddl_dto_action):
        pass

    @abstractmethod
    def _update(self, pddl_dto_action):
        pass

    @abstractmethod
    def save(self, pddl_dto_action):
        pass

    @abstractmethod
    def delete(self, pddl_dto_action):
        pass

    @abstractmethod
    def delete_all(self):
        pass
