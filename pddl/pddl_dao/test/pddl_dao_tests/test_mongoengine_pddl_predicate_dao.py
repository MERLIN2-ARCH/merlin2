
from test_pddl_dao_basic.test_pddl_predicate_dao import TestPddlPredicateDao
from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)


class TestMongoenginePddlPredicateDao(TestPddlPredicateDao):

    def setUp(self):
        super().setUp()
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MONGOENGINE)

        pddl_dao_factory.set_uri("mongodb://localhost:27017/merlin2_tests")

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()


del(TestPddlPredicateDao)
