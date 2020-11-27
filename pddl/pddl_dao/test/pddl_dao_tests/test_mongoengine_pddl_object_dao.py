
from test_pddl_dao_basic.test_pddl_object_dao import TestPddlObjectDao
from pddl_dao.pddl_dao_factory.pddl_dao_factory_factory import PddlDaoFactoryFactory


class TestMongoenginePddlObjectDao(TestPddlObjectDao):

    def setUp(self):
        super().setUp()

        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            pddl_dao_factory_factory.pddl_dao_families.MONGOENGINE)

        pddl_dao_factory.set_uri("mongodb://localhost:27017/merlin2_tests")

        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()
        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()


del(TestPddlObjectDao)
