
import unittest

from merlin2_pddl_generator.merlin2_pddl_generators.mongoengine_merlin2_pddl_generator import(
    MongoengineMerlin2PddlGenerator
)

from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory


class TestMerlin2PddlProblemParser(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        pddl_dao_factory.set_uri("mongodb://localhost:27017/merlin2_tests")
        self.pddl_generator = MongoengineMerlin2PddlGenerator(
            "mongodb://localhost:27017/merlin2_tests")

        # types
        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")

        # objects
        rb1 = PddlObjectDto(robot_type, "rb1")
        wp1 = PddlObjectDto(wp_type, "wp1")
        wp2 = PddlObjectDto(wp_type, "wp2")

        # predicates
        robot_at = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])
        wp_checked = PddlPredicateDto(
            "wp_checked", [robot_type, wp_type])

        # propositions
        rb1_robot_at = PddlPropositionDto(
            robot_at, [rb1, wp1])

        # goals

        rb1_wp2_wp_checked_goal = PddlPropositionDto(
            wp_checked, [rb1, wp2], is_goal=True)

        # actions
        r = PddlObjectDto(robot_type, "r")
        s = PddlObjectDto(wp_type, "s")
        d = PddlObjectDto(wp_type, "d")

        condition_1 = PddlConditionEffectDto(PddlConditionEffectDto.AT_START,
                                             robot_at,
                                             [r, s])

        effect_1 = PddlConditionEffectDto(PddlConditionEffectDto.AT_START,
                                          robot_at,
                                          [r, s],
                                          is_negative=True)

        effect_2 = PddlConditionEffectDto(PddlConditionEffectDto.AT_END,
                                          robot_at,
                                          [r, d])

        navigation_action = PddlActionDto(
            "navigation", [r, s, d], [condition_1], [effect_1, effect_2])

        effect_3 = PddlConditionEffectDto(PddlConditionEffectDto.AT_END,
                                          wp_checked,
                                          [r, s],
                                          is_negative=False)

        check_wp = PddlActionDto(
            "check_wp", [r, s], [condition_1], [effect_3])

        # saving
        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()
        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()
        self.pddl_proposition_dao = pddl_dao_factory.create_pddl_proposition_dao()

        self.pddl_type_dao.save(robot_type)
        self.pddl_type_dao.save(wp_type)

        self.pddl_object_dao.save(rb1)
        self.pddl_object_dao.save(wp1)
        self.pddl_object_dao.save(wp2)

        self.pddl_predicate_dao.save(robot_at)
        self.pddl_predicate_dao.save(wp_checked)

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

    def test_parse_pddl_type_dtos(self):
        self.maxDiff = None

        pddl_generated = self.pddl_generator.generate_pddl()

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
                         pddl_generated[0])

        self.assertEqual("""\
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
                         pddl_generated[1])
