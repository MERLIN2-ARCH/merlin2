
""" Mongoengine Pddl Dao Interface """

from abc import ABC, abstractmethod
from mongoengine import Document
from pddl_dao.mongoengine_pddl_dao.mongoengine_connector import MongoengineConnector
from pddl_dto import PddlDto


class MongoenginePddlDao(ABC):
    """ Mongoengine Pddl Dao Abstract Class """

    def __init__(self, uri: str):

        self.set_uri(uri)
        self.connector.connect()

    def get_uri(self) -> str:
        """ uri getter

        Returns:
            str: Mongo uri
        """

        return self._uri

    def set_uri(self, uri: str):
        """ uri setter

        Args:
            uri (str): Mongo uri
        """

        self._uri = uri

        if self._uri:
            self.connector = MongoengineConnector(uri=self._uri)
        else:
            self.connector = MongoengineConnector()

    @abstractmethod
    def _get_model(self, pddl_dto: PddlDto) -> Document:
        """ get the Mongoengine document corresponding to a give PddlDto

        Args:
            pddl_dto (PddlDto): PddlDto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _model_to_dto(self, pddl_mongoengine: Document) -> PddlDto:
        """ convert a Mongoengine document into a PddlDto

        Args:
            pddl_mongoengine (Document): Mongoengine document

        Returns:
            PddlDto: PddlDto
        """

    @abstractmethod
    def _dto_to_model(self, pddl_dto: PddlDto) -> Document:
        """ convert a PddlDto into a Mongoengine document

        Args:
            pddl_dto (PddlDto): PddlDto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _exist_in_mongo(self, pddl_dto: PddlDto) -> bool:
        """ check if PddlDto exists

        Args:
            pddl_dto (PddlDto): PddlDto

        Returns:
            bool: PddlDto exists?
        """
