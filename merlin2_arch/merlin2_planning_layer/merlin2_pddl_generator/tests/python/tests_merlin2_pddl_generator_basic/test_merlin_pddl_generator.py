import unittest

from merlin2_pddl_generator.merlin2_pddl_generator import Merlin2PddlGenerator

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlActionDto,
)


class TestMerlin2PddlProblemGenerator(unittest.TestCase):

    dao_factory = None

    def setUp(self):

        self.pddl_generator = Merlin2PddlGenerator(self.dao_factory)

        # types
        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")

        # objects
        rb1 = PddlObjectDto(robot_type, "rb1")
        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")

        # predicates
        robot_at = PddlPredicateDto("robot_at", [robot_type, wp_type])
        wp_checked = PddlPredicateDto("wp_checked", [robot_type, wp_type])

        # propositions
        rb1_robot_at = PddlPropositionDto(robot_at, [rb1, wp1])

        # goals

        rb1_wp2_wp_checked_goal = PddlPropositionDto(wp_checked, [rb1, wp2], is_goal=True)

        # actions
        r = PddlObjectDto(robot_type, "r")
        s = PddlObjectDto(wp_type, "s")
        d = PddlObjectDto(wp_type, "d")

        condition_1 = PddlConditionEffectDto(
            robot_at, [r, s], time=PddlConditionEffectDto.AT_START
        )

        effect_1 = PddlConditionEffectDto(
            robot_at, [r, s], time=PddlConditionEffectDto.AT_START, is_negative=True
        )

        effect_2 = PddlConditionEffectDto(
            robot_at, [r, d], time=PddlConditionEffectDto.AT_END
        )

        navigation_action = PddlActionDto(
            "navigation", [r, s, d], [condition_1], [effect_1, effect_2]
        )

        effect_3 = PddlConditionEffectDto(
            wp_checked, [r, s], time=PddlConditionEffectDto.AT_END, is_negative=False
        )

        check_wp = PddlActionDto("check_wp", [r, s], [condition_1], [effect_3])

        # saving
        self.pddl_type_dao = self.dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = self.dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = self.dao_factory.create_pddl_action_dao()
        self.pddl_object_dao = self.dao_factory.create_pddl_object_dao()
        self.pddl_proposition_dao = self.dao_factory.create_pddl_proposition_dao()

        self.pddl_type_dao.save(robot_type)
        self.pddl_type_dao.save(wp_type)

        self.pddl_object_dao.save(rb1)
        self.pddl_object_dao.save(wp1)
        self.pddl_object_dao.save(wp2)

        self.pddl_predicate_dao.save(wp_checked)
        self.pddl_predicate_dao.save(robot_at)

        self.pddl_proposition_dao.save(rb1_robot_at)

        self.pddl_proposition_dao.save(rb1_wp2_wp_checked_goal)

        self.pddl_action_dao.save(navigation_action)
        self.pddl_action_dao.save(check_wp)

    def tearDown(self):
        self.pddl_action_dao.delete_all()
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()
        self.pddl_object_dao.delete_all()
        self.pddl_proposition_dao.delete_all()

    def test_parse_pddl_type_dto_list(self):
        self.maxDiff = None

        pddl_generated = self.pddl_generator.generate_pddl()

        self.assertEqual(
            """\
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
)
""",
            pddl_generated[0],
        )

        self.assertEqual(
            """\
(define (problem merlin2_prb)
(:domain merlin2)
(:objects
\trb1 - robot
\twp1 - wp
\twp2 - wp
)
(:init
\t(robot_at rb1 wp1)
)
(:goal
\t(wp_checked rb1 wp2)
)
)
""",
            pddl_generated[1],
        )
