
from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)

from tests_merlin2_pddl_generator_basic.test_mongoengine_merlin_pddl_generator import TestMerlin2PddlProblemGenerator


class TestMerlin2PddlProblemGeneratorMongoengine(TestMerlin2PddlProblemGenerator):

    def setUp(self):
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        self.pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MONGOENGINE, uri="mongodb://localhost:27017/merlin2_tests")
        super().setUp()


del(TestMerlin2PddlProblemGenerator)
