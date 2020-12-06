
""" Mongoengine Pddl Dao Facory """

from mongoengine import disconnect, connect

from pddl_dao.mongoengine_pddl_dao import (
    MongoenginePddlTypeDao,
    MongoenginePddlObjectDao,
    MongoenginePddlPredicateDao,
    MongoenginePddlPropositionDao,
    MongoenginePddlActionDao
)

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class MongoenginePddlDaoFactory(PddlDaoFactory):
    """ Mongoengine Pddl Dao Facory Class """

    def __init__(self, uri: str = "mongodb://localhost:27017/merlin2"):
        self.set_uri(uri)
        self.connect()

    def connect(self):
        """ connect to current uri
        """

        disconnect()
        connect(host=self._uri)

    def get_uri(self) -> str:
        """ uri getter

        Returns:
            str: uri str
        """

        return self._uri

    def set_uri(self, uri: str):
        """ uri setter

        Args:
            uri (str): uri str
        """

        self._uri = uri

    def create_pddl_type_dao(self) -> MongoenginePddlTypeDao:
        """ create a mongoengine pddl dao type object

        Returns:
            MongoenginePddlTypeDao: mongoengine dao for pddl type
        """

        return MongoenginePddlTypeDao(uri=self._uri, connect=False)

    def create_pddl_predicate_dao(self) -> MongoenginePddlPredicateDao:
        """ create a mongoengine pddl dao predicate object

        Returns:
            MongoenginePddlPredicateDao: mongoengine dao for pddl predicate
        """

        return MongoenginePddlPredicateDao(uri=self._uri, connect=False)

    def create_pddl_action_dao(self) -> MongoenginePddlActionDao:
        """ create a mongoengine pddl dao action object

        Returns:
            MongoenginePddlActionDao: mongoengine dao for pddl action
        """

        return MongoenginePddlActionDao(uri=self._uri, connect=False)

    def create_pddl_object_dao(self) -> MongoenginePddlObjectDao:
        """ create a mongoengine pddl dao object object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlObjectDao: mongoengine dao for pddl object
        """

        return MongoenginePddlObjectDao(uri=self._uri, connect=False)

    def create_pddl_proposition_dao(self) -> MongoenginePddlPropositionDao:
        """ create a mongoengine pddl dao proposition object

        Returns:
            MongoenginePddlPropositionDao: mongoengine dao for pddl proposition
        """

        return MongoenginePddlPropositionDao(uri=self._uri, connect=False)
