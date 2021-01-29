
import unittest
from merlin2_knowledge_base.merlin2_knowledge_base_parser.msg_dto_parser import MsgDtoParser

from merlin2_knowledge_base_interfaces.msg import PddlType
from merlin2_knowledge_base_interfaces.msg import PddlObject
from merlin2_knowledge_base_interfaces.msg import PddlPredicate
from merlin2_knowledge_base_interfaces.msg import PddlProposition
from merlin2_knowledge_base_interfaces.msg import PddlConditionEffect
from merlin2_knowledge_base_interfaces.msg import PddlAction


class TestDtoMsgParser(unittest.TestCase):

    def setUp(self):

        self.parser = MsgDtoParser()

        # types
        self.robot_type = PddlType()
        self.robot_type.type_name = "robot"

        self.wp_type = PddlType()
        self.wp_type.type_name = "wp"

        # objects
        self.rb1 = PddlObject()
        self.rb1.object_name = "rb1"
        self.rb1.pddl_type = self.robot_type

        self.wp1 = PddlObject()
        self.wp1.object_name = "wp1"
        self.wp1.pddl_type = self.wp_type

        self.wp2 = PddlObject()
        self.wp2.object_name = "wp2"
        self.wp2.pddl_type = self.wp_type

        # predicates
        self.robot_at = PddlPredicate()
        self.robot_at.predicate_name = "robot_at"
        self.robot_at.pddl_types = [self.robot_type, self.wp_type]

        self.wp_checked = PddlPredicate()
        self.wp_checked.predicate_name = "wp_checked"
        self.wp_checked.pddl_types = [self.robot_type, self.wp_type]

        # propositions
        self.rb1_robot_at = PddlProposition()
        self.rb1_robot_at.pddl_predicate = self.robot_at
        self.rb1_robot_at.pddl_objects = [self.rb1, self.wp1]
        self.rb1_robot_at.is_goal = False

        # goals
        self.rb1_wp2_wp_checked_goal = PddlProposition()
        self.rb1_wp2_wp_checked_goal.pddl_predicate = self.robot_at
        self.rb1_wp2_wp_checked_goal.pddl_objects = [self.rb1, self.wp2]
        self.rb1_wp2_wp_checked_goal.is_goal = True

        # actions
        r = PddlObject()
        r.object_name = "r"
        r.pddl_type = self.robot_type

        s = PddlObject()
        s.object_name = "s"
        s.pddl_type = self.wp_type

        d = PddlObject()
        d.object_name = "d"
        d.pddl_type = self.wp_type

        condition_1 = PddlConditionEffect()
        condition_1.pddl_predicate = self.robot_at
        condition_1.pddl_objects = [r, s]
        condition_1.time = "at start"
        condition_1.is_negative = False

        effect_1 = PddlConditionEffect()
        effect_1.pddl_predicate = self.robot_at
        effect_1.pddl_objects = [r, s]
        effect_1.time = "at start"
        effect_1.is_negative = True

        effect_2 = PddlConditionEffect()
        effect_2.pddl_predicate = self.robot_at
        effect_2.pddl_objects = [r, d]
        effect_2.time = "at end"
        effect_2.is_negative = False

        self.navigation_action = PddlAction()
        self.navigation_action.duration = 10
        self.navigation_action.durative = True
        self.navigation_action.action_name = "navigation"
        self.navigation_action.pddl_parameters = [r, s, d]
        self.navigation_action.pddl_coditions = [condition_1]
        self.navigation_action.pddl_effects = [effect_1, effect_2]

    def test_parse_type(self):
        dto = self.parser.type_msg_to_dto(self.robot_type)
        self.assertEqual("robot", str(dto))

    def test_parse_object(self):
        dto = self.parser.object_msg_to_dto(self.rb1)
        self.assertEqual("rb1 - robot", str(dto))

    def test_parse_predicate(self):
        dto = self.parser.predicate_msg_to_dto(self.robot_at)
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)", str(dto))

    def test_parse_proposition(self):
        dto = self.parser.proposition_msg_to_dto(self.rb1_robot_at)
        self.assertEqual("(robot_at rb1 wp1)", str(dto))

    def test_parse_proposition_goal(self):
        dto = self.parser.proposition_msg_to_dto(self.rb1_wp2_wp_checked_goal)
        self.assertEqual("(robot_at rb1 wp2)", str(dto))
        self.assertTrue(dto.get_is_goal())

    def test_parse_durative_action(self):
        dto = self.parser.action_msg_to_dto(self.navigation_action)
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

    def test_parse_action(self):
        self.navigation_action.durative = False
        self.navigation_action.pddl_coditions[0].time = ""
        self.navigation_action.pddl_effects[0].time = ""
        self.navigation_action.pddl_effects[1].time = ""

        dto = self.parser.action_msg_to_dto(self.navigation_action)
        self.assertEqual("""\
(:action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:precondition (and
\t\t(robot_at ?r ?s)
\t)
\t:effect (and
\t\t(not (robot_at ?r ?s))
\t\t(robot_at ?r ?d)
\t)
)""",
                         str(dto))
