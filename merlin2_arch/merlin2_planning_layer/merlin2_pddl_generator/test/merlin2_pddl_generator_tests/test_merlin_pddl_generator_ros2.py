
from kant_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)

from simple_node import Node
import rclpy

from tests_merlin2_pddl_generator_basic.test_merlin_pddl_generator import TestMerlin2PddlProblemGenerator


class TestMerlin2PddlProblemGeneratorRos2(TestMerlin2PddlProblemGenerator):

    def setUp(self):

        rclpy.init()
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        self.node = Node("test_mongoengine_merlin_pddl_generator_node")
        self.pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.ROS2, node=self.node)

        super().setUp()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestMerlin2PddlProblemGenerator)
