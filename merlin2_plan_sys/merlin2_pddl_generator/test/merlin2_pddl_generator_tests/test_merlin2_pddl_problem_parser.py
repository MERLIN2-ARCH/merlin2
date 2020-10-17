
import unittest
from merlin2_pddl_generator.merlin2_pddl_parser.merlin2_pddl_problem_parser import Merlin2PddlProblemParser
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto


class TestMerlin2PddlProblemParser(unittest.TestCase):

    def setUp(self):

        self.problem_generator = Merlin2PddlProblemParser()

        # types
        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")

        # objects
        self.rb1 = PddlObjectDto(robot_type, "rb1")
        self.wp1 = PddlObjectDto(wp_type, "wp1")
        self.wp2 = PddlObjectDto(wp_type, "wp2")

        # predicates
        robot_at = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])
        wp_checked = PddlPredicateDto(
            "wp_checked", [robot_type, wp_type])

        # propositions
        self.rb1_robot_at = PddlPropositionDto(
            robot_at, [self.rb1, self.wp1])

        # goals
        self.rb1_robot_at_goal = PddlPropositionDto(
            robot_at, [self.rb1, self.wp2])

        self.rb1_wp2_wp_checked_goal = PddlPropositionDto(
            wp_checked, [self.rb1, self.wp2])

    def test_parse_pddl_type_dtos(self):
        self.maxDiff = None
        self.assertEqual("""\
(:objects
\trb1 - robot
\twp1 - wp
\twp2 - wp
)
""",
                         self.problem_generator.parse_pddl_dto_objects([self.rb1, self.wp1, self.wp2]))

    def test_parse_pddl_type_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("""\
(:objects
)
""",
                         self.problem_generator.parse_pddl_dto_objects([]))

    def test_parse_pddl_propostion_dtos(self):
        self.maxDiff = None
        self.assertEqual("""\
(:init
\t(robot_at rb1 wp1)
)
""",
                         self.problem_generator.parse_pddl_dto_propositions([self.rb1_robot_at]))

    def test_parse_pddl_propostion_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("""\
(:init
)
""",
                         self.problem_generator.parse_pddl_dto_propositions([]))

    def test_parse_pddl_goal_dtos_one_goal(self):
        self.maxDiff = None
        self.assertEqual("""\
(:goal
\t(robot_at rb1 wp2)
)
""",
                         self.problem_generator.parse_pddl_dto_goals([self.rb1_robot_at_goal]))

    def test_parse_pddl_goal_dtos_two_goals(self):
        self.maxDiff = None
        self.assertEqual("""\
(:goal
\t(and
\t(robot_at rb1 wp2)
\t(wp_checked rb1 wp2)
\t)
)
""",
                         self.problem_generator.parse_pddl_dto_goals([self.rb1_robot_at_goal, self.rb1_wp2_wp_checked_goal]))

    def test_parse_pddl_goal_dtos_empty_list(self):
        self.maxDiff = None
        self.assertEqual("""\
(:goal
())
""",
                         self.problem_generator.parse_pddl_dto_goals([]))

    def test_parse_pddl_problem_dto(self):
        self.maxDiff = None
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
                         self.problem_generator.parse_pddl_problem_dto([self.rb1, self.wp1, self.wp2],
                                                                       [self.rb1_robot_at],
                                                                       [self.rb1_wp2_wp_checked_goal]))

    def test_parse_pddl_problem_dto_empty_lists(self):
        self.maxDiff = None
        self.assertEqual("""\
(define (problem merlin2_prb)
(:domain merlin2)
(:objects
)
(:init
)
(:goal
())
)
""",
                         self.problem_generator.parse_pddl_problem_dto([], [], []))
