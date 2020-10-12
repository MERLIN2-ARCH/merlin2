
from abc import ABC, abstractmethod
from pddl_dao.mongoengine_pddl_dao.mongoengine_connector import MongoengineConnector


class Mongoengine_PDDL_DAO(ABC):

    def __init__(self, uri):

        self.set_uri(uri)
        self.connector.connect()

    def get_uri(self):
        return self._uri

    def set_uri(self, uri):
        self._uri = uri

        if(self._uri):
            self.connector = MongoengineConnector(uri=self._uri)
        else:
            self.connector = MongoengineConnector()

    @abstractmethod
    def _get_mongoengine(self, pddl_dto):
        pass

    @abstractmethod
    def _mongoengine_to_dto(self, pddl_mongoengine):
        pass

    @abstractmethod
    def _dto_to_mongoengine(self, pddl_dto):
        pass

    @abstractmethod
    def _exist_in_mongo(self, pddl_dto):
        pass
