
from test_pddl_dao_basic.test_pddl_proposition_dao import TestPddlPropositionDao
from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)


class TestMongoenginePddlPropositionDao(TestPddlPropositionDao):

    def setUp(self):
        super().setUp()
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MONGOENGINE)

        pddl_dao_factory.set_uri("mongodb://localhost:27017/merlin2_tests")
        pddl_dao_factory.connect()

        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()
        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pdd_dao_predicate = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_proposition_dao = pddl_dao_factory.create_pddl_proposition_dao()


del(TestPddlPropositionDao)
