from test_pddl_dao_basic.test_pddl_dao_type import Test_PDDL_DAO_Type
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
import unittest


class Test_Mongoengine_PDDL_DAO_Type(Test_PDDL_DAO_Type):

    def setUp(self):
        super().setUp()
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_pddl_dao_type(
            "mongodb://localhost:27017/merlin2_tests")

    def tearDown(self):
        self.pddl_dao_type.delete_all()


del(Test_PDDL_DAO_Type)
