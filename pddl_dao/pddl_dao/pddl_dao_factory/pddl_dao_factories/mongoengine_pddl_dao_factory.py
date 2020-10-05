from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import Mongoengine_PDDL_DAO_Type
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import Mongoengine_PDDL_DAO_Object
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_proposition import Mongoengine_PDDL_DAO_Proposition
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_action import Mongoengine_PDDL_DAO_Action

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PDDL_DAO_Factory


class Mongoengine_PDDL_DAO_Factory(PDDL_DAO_Factory):

    def create_dao_pddl_type(self, uri=None):
        return Mongoengine_PDDL_DAO_Type(uri=uri)

    def create_dao_pddl_predicate(self, uri=None):
        return Mongoengine_PDDL_DAO_Predicate(uri=uri)

    def create_dao_pddl_action(self, uri=None):
        return Mongoengine_PDDL_DAO_Action(uri=uri)

    def create_dao_pddl_object(self, uri=None):
        return Mongoengine_PDDL_DAO_Object(uri=uri)

    def create_dao_pddl_proposition(self, uri=None):
        return Mongoengine_PDDL_DAO_Proposition(uri=uri)
