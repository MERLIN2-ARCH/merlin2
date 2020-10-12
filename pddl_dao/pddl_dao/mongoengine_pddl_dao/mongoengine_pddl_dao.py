
from abc import ABC, abstractmethod
from pddl_dao.mongoengine_pddl_dao.mongoengine_connector import MongoengineConnector
from mongoengine import Document
from pddl_dao.pddl_dto.pddl_dto import PDDL_DTO


class Mongoengine_PDDL_DAO(ABC):

    def __init__(self, uri: str):

        self.set_uri(uri)
        self.connector.connect()

    def get_uri(self) -> str:
        return self._uri

    def set_uri(self, uri: str):
        self._uri = uri

        if(self._uri):
            self.connector = MongoengineConnector(uri=self._uri)
        else:
            self.connector = MongoengineConnector()

    @abstractmethod
    def _get_mongoengine(self, pddl_dto: PDDL_DTO) -> Document:
        pass

    @abstractmethod
    def _mongoengine_to_dto(self, pddl_mongoengine: Document) -> PDDL_DTO:
        pass

    @abstractmethod
    def _dto_to_mongoengine(self, pddl_dto: PDDL_DTO) -> Document:
        pass

    @abstractmethod
    def _exist_in_mongo(self, pddl_dto: PDDL_DTO) -> bool:
        pass
