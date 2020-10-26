
from test_pddl_dao_basic.test_pddl_object_dao import TestPddlObjectDao
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from threaded_node.node import Node
import rclpy


class TestMerlin2PddlObjectDao(TestPddlObjectDao):

    def setUp(self):
        super().setUp()

        rclpy.init()
        self.node = Node("test_merlin2_pddl_object_dao_node")
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MERLIN2, node=self.node)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
        rclpy.shutdown()


del(TestPddlObjectDao)