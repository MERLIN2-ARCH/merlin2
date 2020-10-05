
from abc import ABC, abstractmethod


class PDDL_DAO_Object:

    @abstractmethod
    def get(self, object_name):
        pass

    @abstractmethod
    def get_all(self):
        pass

    @abstractmethod
    def save(self, pddl_dto_object):
        pass

    @abstractmethod
    def update(self, pddl_dto_object):
        pass

    @abstractmethod
    def save_update(self, pddl_dto_object):
        pass

    @abstractmethod
    def delete(self, pddl_dto_object):
        pass

    @abstractmethod
    def delete_all(self):
        pass
