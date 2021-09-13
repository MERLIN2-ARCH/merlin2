
from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)

from tests_merlin2_pddl_generator_basic.test_merlin_pddl_generator import TestMerlin2PddlProblemGenerator


class TestMerlin2PddlProblemGeneratorMongoengine(TestMerlin2PddlProblemGenerator):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        self.dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO, uri="mongodb://localhost:27017/merlin2_tests")
        super().setUp()


del(TestMerlin2PddlProblemGenerator)
