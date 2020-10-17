
""" Mongoengine Merlin2 Pddl Generator """

from merlin2_pddl_generator.merlin2_pddl_generators.generic_merlin2_pddl_generator import (
    GenericMerlin2PddlGenerator
)

from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory


class MongoengineMerlin2PddlGenerator(GenericMerlin2PddlGenerator):
    """ Mongoengine Merlin2 Pddl Generator Class """

    def __init__(self, uri=None):

        factory_factory = PddlDaoFactoryFactory()
        mongoengine_factory = factory_factory.create_pddl_dao_factory(
            factory_factory.pddl_dao_families.MONGOENGINE)

        mongoengine_factory.set_uri(uri)

        super().__init__(mongoengine_factory)
