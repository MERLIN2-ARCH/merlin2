from test_pddl_dao_basic.test_pddl_dao_proposition import Test_PDDL_DAO_Proposition
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PDDL_DAO_FactoryFactory
import unittest


class Test_Mongoengine_PDDL_DAO_Object(Test_PDDL_DAO_Proposition):

    def setUp(self):
        super().setUp()
        pddl_dao_factory_facory = PDDL_DAO_FactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_object = pddl_dao_factory.create_dao_pddl_object(
            "mongodb://localhost:27017/merlin2")
        self.pddl_dao_type = pddl_dao_factory.create_dao_pddl_type(
            "mongodb://localhost:27017/merlin2")
        self.pdd_dao_predicate = pddl_dao_factory.create_dao_pddl_predicate(
            "mongodb://localhost:27017/merlin2")
        self.pddl_dao_proposition = pddl_dao_factory.create_dao_pddl_proposition(
            "mongodb://localhost:27017/merlin2")


del(Test_PDDL_DAO_Proposition)
