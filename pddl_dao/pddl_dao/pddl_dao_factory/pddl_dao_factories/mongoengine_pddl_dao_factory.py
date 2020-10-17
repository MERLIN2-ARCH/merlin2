
""" Mongoengine Pddl Dao Facory """

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_type_dao import (
    MongoenginePddlTypeDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_object_dao import (
    MongoenginePddlObjectDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_predicate_dao import (
    MongoenginePddlPredicateDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_proposition_dao import (
    MongoenginePddlPropositionDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_action_dao import (
    MongoenginePddlActionDao
)

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class MongoenginePddlDaoFactory(PddlDaoFactory):
    """ Mongoengine Pddl Dao Facory Class """

    def create_pddl_type_dao(self, uri: str = None) -> MongoenginePddlTypeDao:
        """ create a mongoengine pddl dao type object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlTypeDao: mongoengine dao for pddl type
        """

        return MongoenginePddlTypeDao(uri=uri)

    def create_pddl_predicate_dao(self, uri: str = None) -> MongoenginePddlPredicateDao:
        """ create a mongoengine pddl dao predicate object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlPredicateDao: mongoengine dao for pddl predicate
        """

        return MongoenginePddlPredicateDao(uri=uri)

    def create_pddl_action_dao(self, uri: str = None) -> MongoenginePddlActionDao:
        """ create a mongoengine pddl dao action object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlActionDao: mongoengine dao for pddl action
        """

        return MongoenginePddlActionDao(uri=uri)

    def create_pddl_object_dao(self, uri: str = None) -> MongoenginePddlObjectDao:
        """ create a mongoengine pddl dao object object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlObjectDao: mongoengine dao for pddl object
        """

        return MongoenginePddlObjectDao(uri=uri)

    def create_pddl_proposition_dao(self, uri: str = None) -> MongoenginePddlPropositionDao:
        """ create a mongoengine pddl dao proposition object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlPropositionDao: mongoengine dao for pddl proposition
        """

        return MongoenginePddlPropositionDao(uri=uri)
