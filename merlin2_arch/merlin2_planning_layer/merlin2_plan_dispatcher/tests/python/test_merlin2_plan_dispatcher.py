
import unittest
import time
import rclpy

from simple_node import Node

from kant_dao import ParameterLoader
from kant_dto import PddlObjectDto, PddlPropositionDto

from merlin2_arch_interfaces.msg import PlanAction
from merlin2_arch_interfaces.action import DispatchPlan

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_basic_actions.merlin2_navigation_action import Merlin2NavigationAction
from merlin2_plan_dispatcher.merlin2_plan_dispatcher_node import Merlin2PlanDispatcherNode


class FakeNavigationAction(Merlin2NavigationAction):
    def __init__(self):
        super().__init__()
        self.cancel = False

    def run_action(self, goal: PlanAction) -> bool:
        self.cancel = False
        counter = 0
        while not self.cancel and counter < 5:
            time.sleep(1)
            counter += 1

        if self.cancel:
            return False

        return True

    def cancel_action(self):
        self.cancel = True


class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node", namespace="merlin2")

        param_loader = ParameterLoader(self)
        dao_factory = param_loader.get_dao_factory()

        pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()
        self.pddl_action_dao = dao_factory.create_pddl_action_dao()

        self.pddl_proposition_dao.delete_all()
        pddl_object_dao.delete_all()

        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")
        robot_at_prop = PddlPropositionDto(robot_at, [wp1])

        pddl_object_dao.save(wp1)
        pddl_object_dao.save(wp2)
        self.pddl_proposition_dao.save(robot_at_prop)

        self.plan_dispatcher_client = self.create_action_client(
            DispatchPlan, "dispatch_plan")

    def get_propositions(self):
        return self.pddl_proposition_dao.get_all()

    def get_actions(self):
        return self.pddl_action_dao.get_all()

    def call_plan_dispatcher(self):

        goal = DispatchPlan.Goal()

        nav_action = PlanAction()
        nav_action.action_name = "navigation"
        nav_action.objects = ["wp1", "wp2"]

        goal.plan.append(nav_action)

        self.plan_dispatcher_client.wait_for_server()
        self.plan_dispatcher_client.send_goal(goal)

    def wait_plan_dispatcher(self):
        self.plan_dispatcher_client.wait_for_result()

    def cancel_plan_dispatcher(self):
        while not self.plan_dispatcher_client.is_working():
            time.sleep(1)

        return self.plan_dispatcher_client.cancel_goal()

    def is_succeeded(self):
        return self.plan_dispatcher_client.is_succeeded()

    def is_canceled(self):
        return self.plan_dispatcher_client.is_canceled()


class TestMerlin2PlanDispatcher(unittest.TestCase):

    def setUp(self):
        rclpy.init()

        self.fake_action = FakeNavigationAction()
        self.plan_dispatcher = Merlin2PlanDispatcherNode()
        self.client_node = ClientNode()

        super().setUp()

    def tearDown(self):
        super().tearDown()
        rclpy.shutdown()

    def test_plan_dispatcher(self):
        pddl_propositions = self.client_node.get_propositions()
        self.assertEqual(1, len(pddl_propositions))
        self.assertEqual("(robot_at wp1)", str(pddl_propositions[0]))

        while len(self.client_node.get_actions()) == 0:
            time.sleep(1)

        self.client_node.call_plan_dispatcher()
        self.client_node.wait_plan_dispatcher()
        self.assertTrue(self.client_node.is_succeeded())

        pddl_propositions = self.client_node.get_propositions()
        self.assertEqual(1, len(pddl_propositions))
        self.assertEqual("(robot_at wp2)", str(pddl_propositions[0]))

    def test_plan_dispatcher_canceled(self):
        pddl_propositions = self.client_node.get_propositions()
        self.assertEqual(1, len(pddl_propositions))
        self.assertEqual("(robot_at wp1)", str(pddl_propositions[0]))

        while len(self.client_node.get_actions()) == 0:
            time.sleep(1)

        self.client_node.call_plan_dispatcher()
        self.assertTrue(self.client_node.cancel_plan_dispatcher())
        self.client_node.wait_plan_dispatcher()
        self.assertTrue(self.client_node.is_canceled())

        pddl_propositions = self.client_node.get_propositions()
        self.assertEqual(0, len(pddl_propositions))
