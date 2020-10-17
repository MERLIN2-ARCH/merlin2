
from test_pddl_dao_basic.test_pddl_dao_type import TestPddlDaoType
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory


class TestMongoenginePddlDaoType(TestPddlDaoType):

    def setUp(self):
        super().setUp()
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao(
            "mongodb://localhost:27017/merlin2_tests")

    def tearDown(self):
        self.pddl_type_dao.delete_all()


del(TestPddlDaoType)
