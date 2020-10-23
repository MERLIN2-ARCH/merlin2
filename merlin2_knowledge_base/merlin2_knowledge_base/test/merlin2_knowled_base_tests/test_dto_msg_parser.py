
import unittest
from merlin2_knowledge_base.merlin2_knowledge_base_parser.dto_msg_parser import DtoMsgParser

from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto


class TestDtoMsgParser(unittest.TestCase):

    def setUp(self):

        self.parser = DtoMsgParser()

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

    def test_parse_type(self):
        msg = self.parser.type_dto_to_msg(self.robot_type)
        self.assertEqual("robot", msg.type_name)

    def test_parse_object(self):
        msg = self.parser.object_dto_to_msg(self.rb1)
        self.assertEqual("rb1", msg.object_name)
        self.assertEqual("robot", msg.pddl_type.type_name)

    def test_parse_predicate(self):
        msg = self.parser.predicate_dto_to_msg(self.robot_at)
        self.assertEqual("robot_at", msg.predicate_name)
        self.assertEqual("robot", msg.pddl_types[0].type_name)
        self.assertEqual("wp", msg.pddl_types[1].type_name)

    def test_parse_proposition(self):
        msg = self.parser.proposition_dto_to_msg(self.rb1_robot_at)
        self.assertEqual("robot_at", msg.pddl_predicate.predicate_name)
        self.assertEqual("rb1", msg.pddl_objects[0].object_name)
        self.assertEqual("wp1", msg.pddl_objects[1].object_name)
        self.assertFalse(msg.is_goal)

    def test_parse_proposition_goal(self):
        msg = self.parser.proposition_dto_to_msg(self.rb1_wp2_wp_checked_goal)
        self.assertEqual("wp_checked", msg.pddl_predicate.predicate_name)
        self.assertEqual("rb1", msg.pddl_objects[0].object_name)
        self.assertEqual("wp2", msg.pddl_objects[1].object_name)
        self.assertTrue(msg.is_goal)

    def test_parse_action(self):
        msg = self.parser.action_dto_to_msg(self.navigation_action)
        self.assertEqual("navigation", msg.action_name)

        self.assertEqual("robot", msg.pddl_parameters[0].pddl_type.type_name)
        self.assertEqual("r", msg.pddl_parameters[0].object_name)
        self.assertEqual("robot", msg.pddl_parameters[0].pddl_type.type_name)
        self.assertEqual("s", msg.pddl_parameters[1].object_name)
        self.assertEqual("wp", msg.pddl_parameters[1].pddl_type.type_name)
        self.assertEqual("d", msg.pddl_parameters[2].object_name)
        self.assertEqual("wp", msg.pddl_parameters[2].pddl_type.type_name)

        self.assertEqual(
            "robot_at", msg.pddl_coditions[0].pddl_proposition.pddl_predicate.predicate_name)
        self.assertFalse(msg.pddl_coditions[0].is_negative)
        self.assertEqual(
            "r", msg.pddl_coditions[0].pddl_proposition.pddl_objects[0].object_name)
        self.assertEqual(
            "robot", msg.pddl_coditions[0].pddl_proposition.pddl_objects[0].pddl_type.type_name)
        self.assertEqual(
            "s", msg.pddl_coditions[0].pddl_proposition.pddl_objects[1].object_name)
        self.assertEqual(
            "wp", msg.pddl_coditions[0].pddl_proposition.pddl_objects[1].pddl_type.type_name)

        self.assertEqual(
            "robot_at", msg.pddl_effects[0].pddl_proposition.pddl_predicate.predicate_name)
        self.assertTrue(msg.pddl_effects[0].is_negative)
        self.assertEqual(
            "r", msg.pddl_effects[0].pddl_proposition.pddl_objects[0].object_name)
        self.assertEqual(
            "robot", msg.pddl_effects[0].pddl_proposition.pddl_objects[0].pddl_type.type_name)
        self.assertEqual(
            "s", msg.pddl_effects[0].pddl_proposition.pddl_objects[1].object_name)
        self.assertEqual(
            "wp", msg.pddl_effects[0].pddl_proposition.pddl_objects[1].pddl_type.type_name)

        self.assertEqual(
            "robot_at", msg.pddl_effects[1].pddl_proposition.pddl_predicate.predicate_name)
        self.assertFalse(msg.pddl_effects[1].is_negative)
        self.assertEqual(
            "r", msg.pddl_effects[1].pddl_proposition.pddl_objects[0].object_name)
        self.assertEqual(
            "robot", msg.pddl_effects[1].pddl_proposition.pddl_objects[0].pddl_type.type_name)
        self.assertEqual(
            "d", msg.pddl_effects[1].pddl_proposition.pddl_objects[1].object_name)
        self.assertEqual(
            "wp", msg.pddl_effects[1].pddl_proposition.pddl_objects[1].pddl_type.type_name)
