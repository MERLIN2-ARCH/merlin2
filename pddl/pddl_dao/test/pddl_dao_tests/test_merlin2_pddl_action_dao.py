
from test_pddl_dao_basic.test_pddl_action_dao import TestPddlActionDao
from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)
from custom_ros2 import Node
import rclpy


class TestMerlin2PddlActionDao(TestPddlActionDao):

    def setUp(self):
        super().setUp()

        rclpy.init()
        self.node = Node("test_merlin2_pddl_type_dao_node")
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            PddlDaoFamilies.MERLIN2, node=self.node)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestPddlActionDao)
