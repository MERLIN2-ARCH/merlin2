
import unittest
from merlin2_knowledge_base.merlin2_knowledge_base.merlin2_konwledge_base import Merlin2KnowledgeBase

from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto


class TestDtoMsgParser(unittest.TestCase):

    def setUp(self):

        self.knowledge_nase = Merlin2KnowledgeBase()

        # types
        self.robot_type = PddlTypeDto("robot")
        self.wp_type = PddlTypeDto("wp")

        # objects
        self.rb1 = PddlObjectDto(self.robot_type, "rb1")
        self.wp1 = PddlObjectDto(self.wp_type, "wp1")
        self.wp2 = PddlObjectDto(self.wp_type, "wp2")

        # predicates
        self.robot_at = PddlPredicateDto(
            "robot_at", [self.robot_type, self.wp_type])
        self.wp_checked = PddlPredicateDto(
            "wp_checked", [self.robot_type, self.wp_type])

        # propositions
        self.rb1_robot_at = PddlPropositionDto(
            self.robot_at, [self.rb1, self.wp1])

        # goals
        self.rb1_wp2_wp_checked_goal = PddlPropositionDto(
            self.wp_checked, [self.rb1, self.wp2], is_goal=True)

        # actions
        r = PddlObjectDto(self.robot_type, "r")
        s = PddlObjectDto(self.wp_type, "s")
        d = PddlObjectDto(self.wp_type, "d")

        condition_1 = PddlConditionEffectDto(self.robot_at,
                                             [r, s],
                                             time=PddlConditionEffectDto.AT_START)

        effect_1 = PddlConditionEffectDto(self.robot_at,
                                          [r, s],
                                          time=PddlConditionEffectDto.AT_START,
                                          is_negative=True)

        effect_2 = PddlConditionEffectDto(self.robot_at,
                                          [r, d],
                                          time=PddlConditionEffectDto.AT_END)

        self.navigation_action = PddlActionDto(
            "navigation", [r, s, d], [condition_1], [effect_1, effect_2])

    def tearDown(self):
        self.knowledge_nase.delete_all_actions()
        self.knowledge_nase.delete_all_objects()
        self.knowledge_nase.delete_all_propositions()
        self.knowledge_nase.delete_all_predicates()
        self.knowledge_nase.delete_all_types()

    def test_save_type_true(self):
        succ = self.knowledge_nase.save_type(self.robot_type)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_type("robot")
        self.assertEqual("robot", str(dto))
        self.assertEqual(1, len(self.knowledge_nase.get_all_types()))

    def test_delete_type_true(self):
        self.knowledge_nase.save_type(self.robot_type)
        self.assertEqual(1, len(self.knowledge_nase.get_all_types()))
        succ = self.knowledge_nase.delete_type(self.robot_type)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_types()))

    def test_delete_type_false(self):
        succ = self.knowledge_nase.delete_type(self.robot_type)
        self.assertFalse(succ)

    def test_delete_all_types(self):
        self.knowledge_nase.save_type(self.robot_type)
        self.assertEqual(1, len(self.knowledge_nase.get_all_types()))
        succ = self.knowledge_nase.delete_all_types()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_types()))

    def test_save_object_true(self):
        succ = self.knowledge_nase.save_object(self.rb1)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_object("rb1")
        self.assertEqual("rb1 - robot", str(dto))
        self.assertEqual(1, len(self.knowledge_nase.get_all_objects()))

    def test_delete_object_true(self):
        self.knowledge_nase.save_object(self.rb1)
        self.assertEqual(1, len(self.knowledge_nase.get_all_objects()))
        succ = self.knowledge_nase.delete_object(self.rb1)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_objects()))

    def test_delete_object_false(self):
        succ = self.knowledge_nase.delete_object(self.rb1)
        self.assertFalse(succ)

    def test_delete_all_objects(self):
        self.knowledge_nase.save_object(self.rb1)
        self.assertEqual(1, len(self.knowledge_nase.get_all_objects()))
        succ = self.knowledge_nase.delete_all_objects()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_objects()))

    def test_save_predicate_true(self):
        succ = self.knowledge_nase.save_predicate(self.robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_predicate("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)", str(dto))
        self.assertEqual(1, len(self.knowledge_nase.get_all_predicates()))

    def test_delete_predicate_true(self):
        self.knowledge_nase.save_predicate(self.robot_at)
        self.assertEqual(1, len(self.knowledge_nase.get_all_predicates()))
        succ = self.knowledge_nase.delete_predicate(self.robot_at)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_predicates()))

    def test_delete_predicate_false(self):
        succ = self.knowledge_nase.delete_predicate(self.robot_at)
        self.assertFalse(succ)

    def test_delete_all_predicates(self):
        self.knowledge_nase.save_predicate(self.robot_at)
        self.assertEqual(1, len(self.knowledge_nase.get_all_predicates()))
        succ = self.knowledge_nase.delete_all_predicates()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_predicates()))

    def test_save_proposition_no_goal(self):
        succ = self.knowledge_nase.save_proposition(self.rb1_robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_propositions_no_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)", str(dto))
        self.assertEqual(
            1, len(self.knowledge_nase.get_propositions_no_goals()))

    def test_save_proposition_goal(self):
        succ = self.knowledge_nase.save_proposition(
            self.rb1_wp2_wp_checked_goal)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_propositions_goals()[0]
        self.assertEqual("(wp_checked rb1 wp2)", str(dto))
        self.assertEqual(
            1, len(self.knowledge_nase.get_propositions_goals()))

    def test_delete_proposition_true(self):
        self.knowledge_nase.save_proposition(self.rb1_robot_at)
        self.assertEqual(1, len(self.knowledge_nase.get_all_propositions()))
        succ = self.knowledge_nase.delete_proposition(self.rb1_robot_at)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_propositions()))

    def test_delete_proposition_false(self):
        succ = self.knowledge_nase.delete_proposition(self.rb1_robot_at)
        self.assertFalse(succ)

    def test_delete_all_propositions(self):
        self.knowledge_nase.save_proposition(self.rb1_robot_at)
        self.assertEqual(1, len(self.knowledge_nase.get_all_propositions()))
        succ = self.knowledge_nase.delete_all_propositions()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_propositions()))

    def test_save_action_true(self):
        succ = self.knowledge_nase.save_action(self.navigation_action)
        self.assertTrue(succ)
        dto = self.knowledge_nase.get_action("navigation")
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at start (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(dto))
        self.assertEqual(1, len(self.knowledge_nase.get_all_actions()))

    def test_delete_action_true(self):
        self.knowledge_nase.save_action(self.navigation_action)
        self.assertEqual(1, len(self.knowledge_nase.get_all_actions()))
        succ = self.knowledge_nase.delete_action(self.navigation_action)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_actions()))

    def test_delete_action_false(self):
        succ = self.knowledge_nase.delete_action(self.navigation_action)
        self.assertFalse(succ)

    def test_delete_all_actions(self):
        self.knowledge_nase.save_action(self.navigation_action)
        self.assertEqual(1, len(self.knowledge_nase.get_all_actions()))
        succ = self.knowledge_nase.delete_all_actions()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_nase.get_all_actions()))
