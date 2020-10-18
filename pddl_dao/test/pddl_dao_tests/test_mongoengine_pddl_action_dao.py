
from test_pddl_dao_basic.test_pddl_action_dao import TestPddlActionDao
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory


class TestMongoenginePddlActionDao(TestPddlActionDao):

    def setUp(self):
        super().setUp()

        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE, uri="mongodb://localhost:27017/merlin2_tests")

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pdd_dao_predicate = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()


del(TestPddlActionDao)
