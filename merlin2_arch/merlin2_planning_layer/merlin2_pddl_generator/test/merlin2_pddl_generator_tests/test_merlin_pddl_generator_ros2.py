
from kant_dao.dao_factory import (
    DaoFactoryFactory,
    DaoFamilies
)

from simple_node import Node
import rclpy

from tests_merlin2_pddl_generator_basic.test_merlin_pddl_generator import TestMerlin2PddlProblemGenerator


class TestMerlin2PddlProblemGeneratorRos2(TestMerlin2PddlProblemGenerator):

    def setUp(self):

        rclpy.init()
        dao_factory_factory = DaoFactoryFactory()
        self.node = Node("test_mongoengine_merlin_pddl_generator_node")
        self.dao_factory = dao_factory_factory.create_dao_factory(
            DaoFamilies.ROS2, node=self.node)

        super().setUp()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestMerlin2PddlProblemGenerator)
