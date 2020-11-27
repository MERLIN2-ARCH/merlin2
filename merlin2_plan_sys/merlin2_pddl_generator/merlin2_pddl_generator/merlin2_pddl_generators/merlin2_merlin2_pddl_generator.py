
""" Merlin2 Merlin2 Pddl Generator """

from rclpy.node import Node

from merlin2_pddl_generator.merlin2_pddl_generators import (
    Merlin2PddlGenerator
)

from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)


class Merlin2Merlin2PddlGenerator(Merlin2PddlGenerator):
    """ Merlin2 Merlin2 Pddl Generator Class """

    def __init__(self, node: Node):

        factory_factory = PddlDaoFactoryFactory()
        merlin2_factory = factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MERLIN2, node=node)

        super().__init__(merlin2_factory)
