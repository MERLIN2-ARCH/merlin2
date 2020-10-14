
""" Mongoengine Pddl Dao Facory """

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import (
    MongoenginePddlDaoType
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import (
    Mongoengine_PDDL_DAO_Object
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import (
    Mongoengine_PDDL_DAO_Predicate
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_proposition import (
    MongoenginePddlDaoProposition
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_action import (
    Mongoengine_PDDL_DAO_Action
)

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class MongoenginePddlDaoFactory(PddlDaoFactory):
    """ Mongoengine Pddl Dao Facory Class """

    def create_pddl_dao_type(self, uri: str = None) -> MongoenginePddlDaoType:
        """ create a mongoengine pddl dao type object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlDaoType: mongoengine dao for pddl type
        """

        return MongoenginePddlDaoType(uri=uri)

    def create_pddl_dao_predicate(self, uri: str = None) -> Mongoengine_PDDL_DAO_Predicate:
        """ create a mongoengine pddl dao predicate object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Mongoengine_PDDL_DAO_Predicate: mongoengine dao for pddl predicate
        """

        return Mongoengine_PDDL_DAO_Predicate(uri=uri)

    def create_pddl_dao_action(self, uri: str = None) -> Mongoengine_PDDL_DAO_Action:
        """ create a mongoengine pddl dao action object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Mongoengine_PDDL_DAO_Action: mongoengine dao for pddl action
        """

        return Mongoengine_PDDL_DAO_Action(uri=uri)

    def create_pddl_dao_object(self, uri: str = None) -> Mongoengine_PDDL_DAO_Object:
        """ create a mongoengine pddl dao object object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Mongoengine_PDDL_DAO_Object: mongoengine dao for pddl object
        """

        return Mongoengine_PDDL_DAO_Object(uri=uri)

    def create_pddl_dao_proposition(self, uri: str = None) -> MongoenginePddlDaoProposition:
        """ create a mongoengine pddl dao proposition object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoenginePddlDaoProposition: mongoengine dao for pddl proposition
        """

        return MongoenginePddlDaoProposition(uri=uri)
