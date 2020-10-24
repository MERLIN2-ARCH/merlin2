
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

        self.knowledge_base = Merlin2KnowledgeBase()

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

        self.empty_wp = PddlPredicateDto(
            "empty_wp", [self.wp_type])

        # propositions
        self.rb1_robot_at = PddlPropositionDto(
            self.robot_at, [self.rb1, self.wp1])
        self.wp1_empty_wp = PddlPropositionDto(
            self.empty_wp, [self.wp1])

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

        w = PddlObjectDto(self.wp_type, "w")
        effect_3 = PddlConditionEffectDto(self.empty_wp,
                                          [w],
                                          time=PddlConditionEffectDto.AT_START,
                                          is_negative=True)
        self.empty_wp_action = PddlActionDto(
            "empty_wp", [w], [], [effect_3])

    def tearDown(self):
        self.knowledge_base.delete_all_actions()
        self.knowledge_base.delete_all_objects()
        self.knowledge_base.delete_all_propositions()
        self.knowledge_base.delete_all_predicates()
        self.knowledge_base.delete_all_types()

    def test_save_type_true(self):
        succ = self.knowledge_base.save_type(self.robot_type)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_type("robot")
        self.assertEqual("robot", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_types()))

    def test_delete_type_true(self):
        self.knowledge_base.save_type(self.robot_type)
        self.assertEqual(1, len(self.knowledge_base.get_all_types()))
        succ = self.knowledge_base.delete_type(self.robot_type)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_types()))

    def test_delete_type_false(self):
        succ = self.knowledge_base.delete_type(self.robot_type)
        self.assertFalse(succ)

    def test_delete_all_types(self):
        self.knowledge_base.save_type(self.robot_type)
        self.assertEqual(1, len(self.knowledge_base.get_all_types()))
        succ = self.knowledge_base.delete_all_types()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_types()))

    def test_save_object_true(self):
        succ = self.knowledge_base.save_object(self.rb1)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_object("rb1")
        self.assertEqual("rb1 - robot", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_objects()))
        dto = self.knowledge_base.get_type("robot")
        self.assertEqual("robot", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_types()))

    def test_save_object_true_type_alrready_exists(self):
        self.knowledge_base.save_type(self.robot_type)
        succ = self.knowledge_base.save_object(self.rb1)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_object("rb1")
        self.assertEqual("rb1 - robot", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_objects()))

    def test_delete_object_true(self):
        self.knowledge_base.save_object(self.rb1)
        self.assertEqual(1, len(self.knowledge_base.get_all_objects()))
        succ = self.knowledge_base.delete_object(self.rb1)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_objects()))

    def test_delete_object_false(self):
        succ = self.knowledge_base.delete_object(self.rb1)
        self.assertFalse(succ)

    def test_delete_all_objects(self):
        self.knowledge_base.save_object(self.rb1)
        self.assertEqual(1, len(self.knowledge_base.get_all_objects()))
        succ = self.knowledge_base.delete_all_objects()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_objects()))

    def test_save_predicate_true(self):
        succ = self.knowledge_base.save_predicate(self.robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_predicate("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))
        dto = self.knowledge_base.get_type("robot")
        self.assertEqual("robot", str(dto))
        dto = self.knowledge_base.get_type("wp")
        self.assertEqual("wp", str(dto))
        self.assertEqual(2, len(self.knowledge_base.get_all_types()))

    def test_save_predicate_true_type_alrready_exists(self):
        self.knowledge_base.save_type(self.robot_type)
        self.knowledge_base.save_type(self.wp_type)
        succ = self.knowledge_base.save_predicate(self.robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_predicate("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))

    def test_delete_predicate_true(self):
        self.knowledge_base.save_predicate(self.robot_at)
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))
        succ = self.knowledge_base.delete_predicate(self.robot_at)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_predicates()))

    def test_delete_predicate_false(self):
        succ = self.knowledge_base.delete_predicate(self.robot_at)
        self.assertFalse(succ)

    def test_delete_all_predicates(self):
        self.knowledge_base.save_predicate(self.robot_at)
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))
        succ = self.knowledge_base.delete_all_predicates()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_predicates()))

    def test_save_proposition_no_goal_true(self):
        succ = self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_propositions_no_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)", str(dto))
        self.assertEqual(
            1, len(self.knowledge_base.get_propositions_no_goals()))
        dto = self.knowledge_base.get_object("rb1")
        self.assertEqual("rb1 - robot", str(dto))
        dto = self.knowledge_base.get_object("wp1")
        self.assertEqual("wp1 - wp", str(dto))
        self.assertEqual(2, len(self.knowledge_base.get_all_objects()))

    def test_save_proposition_no_goal_true_objects_alrready_exists(self):
        self.knowledge_base.save_object(self.rb1)
        self.knowledge_base.save_object(self.wp1)
        succ = self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_propositions_no_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)", str(dto))
        self.assertEqual(
            1, len(self.knowledge_base.get_propositions_no_goals()))

    def test_save_proposition_no_goal_false_bad_types(self):
        self.robot_at.get_pddl_types_list().reverse()
        succ = self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertFalse(succ)
        self.assertEqual(
            0, len(self.knowledge_base.get_propositions_no_goals()))

    def test_save_proposition_no_goal_false_bad_len(self):
        self.robot_at.set_pddl_types_list([])
        succ = self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertFalse(succ)
        self.assertEqual(
            0, len(self.knowledge_base.get_propositions_no_goals()))

    def test_save_proposition_goal_true(self):
        succ = self.knowledge_base.save_proposition(
            self.rb1_wp2_wp_checked_goal)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_propositions_goals()[0]
        self.assertEqual("(wp_checked rb1 wp2)", str(dto))
        self.assertEqual(
            1, len(self.knowledge_base.get_propositions_goals()))

    def test_delete_proposition_goal_true(self):
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertEqual(1, len(self.knowledge_base.get_all_propositions()))
        succ = self.knowledge_base.delete_proposition(self.rb1_robot_at)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))

    def test_delete_proposition_goal_false(self):
        succ = self.knowledge_base.delete_proposition(self.rb1_robot_at)
        self.assertFalse(succ)

    def test_delete_proposition_no_goal_true(self):
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.assertEqual(1, len(self.knowledge_base.get_all_propositions()))
        succ = self.knowledge_base.delete_proposition(
            self.rb1_wp2_wp_checked_goal)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))

    def test_delete_proposition_no_goal_false(self):
        succ = self.knowledge_base.delete_proposition(
            self.rb1_wp2_wp_checked_goal)
        self.assertFalse(succ)

    def test_delete_all_propositions(self):
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.assertEqual(1, len(self.knowledge_base.get_all_propositions()))
        succ = self.knowledge_base.delete_all_propositions()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))

    def test_save_action_true(self):
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_action("navigation")
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
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))
        dto = self.knowledge_base.get_predicate("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)", str(dto))
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))

    def test_save_action_true_types_predicates_alrready_exists(self):
        self.knowledge_base.save_type(self.robot_type)
        self.knowledge_base.save_type(self.wp_type)
        self.knowledge_base.save_predicate(self.robot_at)
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertTrue(succ)
        dto = self.knowledge_base.get_action("navigation")
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
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))

    def test_save_action_false_bad_condition_types(self):
        self.navigation_action.get_conditions_list(
        )[0].get_pddl_objects_list().reverse()
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertFalse(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_save_action_false_bad_condition_len(self):
        self.navigation_action.get_conditions_list(
        )[0].set_pddl_objects_list([])
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertFalse(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_save_action_false_bad_effect_types(self):
        self.navigation_action.get_effects_list(
        )[0].get_pddl_objects_list().reverse()
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertFalse(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_save_action_false_bad_effect_len(self):
        self.navigation_action.get_effects_list(
        )[0].set_pddl_objects_list([])
        succ = self.knowledge_base.save_action(self.navigation_action)
        self.assertFalse(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_delete_action_true(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))
        succ = self.knowledge_base.delete_action(self.navigation_action)
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_delete_action_false(self):
        succ = self.knowledge_base.delete_action(self.navigation_action)
        self.assertFalse(succ)

    def test_delete_all_actions(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))
        succ = self.knowledge_base.delete_all_actions()
        self.assertTrue(succ)
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))

    def test_delete_type_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_type(self.robot_type)
        self.assertTrue(succ)

        self.assertEqual(1, len(self.knowledge_base.get_all_types()))
        self.assertEqual(1, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(2, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(1, len(self.knowledge_base.get_all_propositions()))

    def test_delete_all_types_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_all_types()
        self.assertTrue(succ)

        self.assertEqual(0, len(self.knowledge_base.get_all_types()))
        self.assertEqual(0, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(0, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))

    def test_delete_object_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_object(self.rb1)
        self.assertTrue(succ)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(2, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(1, len(self.knowledge_base.get_all_propositions()))

    def test_delete_all_objects_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_all_objects()
        self.assertTrue(succ)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(0, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))

    def test_delete_predicate_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_predicate(self.robot_at)
        self.assertTrue(succ)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(2, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(1, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(2, len(self.knowledge_base.get_all_propositions()))

    def test_delete_all_predicates_propagating_deleting(self):
        self.knowledge_base.save_action(self.navigation_action)
        self.knowledge_base.save_action(self.empty_wp_action)
        self.knowledge_base.save_proposition(self.rb1_robot_at)
        self.knowledge_base.save_proposition(self.rb1_wp2_wp_checked_goal)
        self.knowledge_base.save_proposition(self.wp1_empty_wp)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(3, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(2, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(3, len(self.knowledge_base.get_all_propositions()))

        succ = self.knowledge_base.delete_all_predicates()
        self.assertTrue(succ)

        self.assertEqual(2, len(self.knowledge_base.get_all_types()))
        self.assertEqual(0, len(self.knowledge_base.get_all_predicates()))
        self.assertEqual(0, len(self.knowledge_base.get_all_actions()))
        self.assertEqual(3, len(self.knowledge_base.get_all_objects()))
        self.assertEqual(0, len(self.knowledge_base.get_all_propositions()))
