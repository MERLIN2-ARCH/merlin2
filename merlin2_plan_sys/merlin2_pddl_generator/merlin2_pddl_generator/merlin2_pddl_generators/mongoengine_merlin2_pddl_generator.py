
""" Mongoengine Merlin2 Pddl Generator """

from merlin2_pddl_generator.merlin2_pddl_generators import (
    Merlin2PddlGenerator
)

from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)


class MongoengineMerlin2PddlGenerator(Merlin2PddlGenerator):
    """ Mongoengine Merlin2 Pddl Generator Class """

    def __init__(self, uri=None):

        factory_factory = PddlDaoFactoryFactory()
        mongoengine_factory = factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MONGOENGINE)

        mongoengine_factory.set_uri(uri)

        super().__init__(mongoengine_factory)
