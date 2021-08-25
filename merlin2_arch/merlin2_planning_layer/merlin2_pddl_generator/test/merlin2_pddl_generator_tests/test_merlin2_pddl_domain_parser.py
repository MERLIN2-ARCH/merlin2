
import unittest
from merlin2_pddl_generator.merlin2_pddl_parser import Merlin2PddlDomainParser

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlConditionEffectDto,
    PddlActionDto
)


class TestMerlin2PddlDomainParser(unittest.TestCase):

    def setUp(self):

        self.domain_parser = Merlin2PddlDomainParser()

        # types
        self.robot_type = PddlTypeDto("robot")
        self.wp_type = PddlTypeDto("wp")

        # predicates
        self.robot_at = PddlPredicateDto(
            "robot_at", [self.robot_type, self.wp_type])
        self.wp_checked = PddlPredicateDto(
            "wp_checked", [self.robot_type, self.wp_type])

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

        effect_3 = PddlConditionEffectDto(self.wp_checked,
                                          [r, s],
                                          time=PddlConditionEffectDto.AT_END,
                                          is_negative=False)

        self.check_wp = PddlActionDto(
            "check_wp", [r, s], [condition_1], [effect_3])

    def test_parse_pddl_type_dtos(self):
        self.maxDiff = None
        self.assertEqual("""\
(:types
\trobot
\twp
)
""",
                         self.domain_parser.parse_pddl_type_dtos([self.robot_type, self.wp_type]))

    def test_parse_pddl_type_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("""\
(:types
)
""",
                         self.domain_parser.parse_pddl_type_dtos([]))

    def test_parse_pddl_predicate_dtos(self):
        self.maxDiff = None
        self.assertEqual("""\
(:predicates
\t(robot_at ?r0 - robot ?w1 - wp)
\t(wp_checked ?r0 - robot ?w1 - wp)
)
""",
                         self.domain_parser.parse_pddl_predicate_dtos([self.robot_at, self.wp_checked]))

    def test_parse_pddl_predicate_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("""\
(:predicates
)
""",
                         self.domain_parser.parse_pddl_predicate_dtos([]))

    def test_parse_pddl_action_dtos(self):
        self.maxDiff = None
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
)
(:durative-action check_wp
\t:parameters ( ?r - robot ?s - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at end (wp_checked ?r ?s))
\t)
)
""",
                         self.domain_parser.parse_pddl_action_dtos([self.navigation_action, self.check_wp]))

    def test_parse_pddl_action_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("",
                         self.domain_parser.parse_pddl_action_dtos([]))

    def test_parse_pddl_domain_dto(self):
        self.maxDiff = None
        self.assertEqual("""\
(define (domain merlin2)
(:requirements :typing :negative-preconditions :durative-actions)
(:types
\trobot
\twp
)
(:predicates
\t(robot_at ?r0 - robot ?w1 - wp)
\t(wp_checked ?r0 - robot ?w1 - wp)
)
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
)
(:durative-action check_wp
\t:parameters ( ?r - robot ?s - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at end (wp_checked ?r ?s))
\t)
)
)
""",
                         self.domain_parser.parse_pddl_domain_dto([self.robot_type, self.wp_type],
                                                                  [self.robot_at,
                                                                   self.wp_checked],
                                                                  [self.navigation_action, self.check_wp]))

    def test_parse_pddl_domain_dto_empty_lists(self):
        self.maxDiff = None
        self.assertEqual("""\
(define (domain merlin2)
(:requirements :typing :negative-preconditions :durative-actions)
(:types
)
(:predicates
)
)
""",
                         self.domain_parser.parse_pddl_domain_dto([], [], []))
