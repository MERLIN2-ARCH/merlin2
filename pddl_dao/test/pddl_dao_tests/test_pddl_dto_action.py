import unittest
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto


class TestPddlDtoAction(unittest.TestCase):

    def setUp(self):

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto(
            "robot_at", [self._robot_type, self._wp_type])

        r = PddlObjectDto(self._robot_type, "r")
        s = PddlObjectDto(self._wp_type, "s")
        d = PddlObjectDto(self._wp_type, "d")

        self._condition_1 = PddlConditionEffectDto(PddlConditionEffectDto.AT_START,
                                                   self._robot_at,
                                                   [r, s])

        self._effect_1 = PddlConditionEffectDto(PddlConditionEffectDto.AT_START,
                                                self._robot_at,
                                                [r, s],
                                                is_negative=True)

        self._effect_2 = PddlConditionEffectDto(PddlConditionEffectDto.AT_END,
                                                self._robot_at,
                                                [r, d])

        self.pddl_action_dto = PddlActionDto(
            "navigation", [r, s, d], [self._condition_1], [self._effect_1, self._effect_2])

    def test_pddl_dto_action_str(self):
        self.maxDiff = None
        self.pddl_action_dto.set_durative(False)
        self.assertEqual("""\
(:action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:precondition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at start (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dto_action_str_durative(self):
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
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dto_action_str_durative_no_effects(self):
        self.maxDiff = None
        self.pddl_action_dto.set_effects_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dto_action_str_durative_no_conditions(self):
        self.maxDiff = None
        self.pddl_action_dto.set_conditions_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t)
\t:effect (and
\t\t(at start (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dto_action_str_durative_no_parameters(self):
        self.maxDiff = None
        self.pddl_action_dto.set_conditions_list([])
        self.pddl_action_dto.set_effects_list([])
        self.pddl_action_dto.set_parameters_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ()
\t:duration (= ?duration 10)
\t:condition (and
\t)
\t:effect (and
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dto_action_get_action_name(self):
        self.assertEqual("navigation", self.pddl_action_dto.get_action_name())

    def test_pddl_dto_action_get_prameters_list(self):
        params_list = self.pddl_action_dto.get_parameters_list()
        self.assertEqual("r - robot", str(params_list[0]))
        self.assertEqual("s - wp", str(params_list[1]))
        self.assertEqual("d - wp", str(params_list[2]))

    def test_pddl_dto_action_get_conditions_list(self):
        conditions_list = self.pddl_action_dto.get_conditions_list()
        self.assertEqual("(at start (robot_at ?r ?s))",
                         str(conditions_list[0]))

    def test_pddl_dto_action_get_effects_list(self):
        effects_list = self.pddl_action_dto.get_effects_list()
        self.assertEqual("(at start (not (robot_at ?r ?s)))",
                         str(effects_list[0]))
        self.assertEqual("(at end (robot_at ?r ?d))",
                         str(effects_list[1]))

    def test_pddl_dto_action_get_durative(self):
        self.assertTrue(self.pddl_action_dto.get_durative())

    def test_pddl_dto_action_get_duration(self):
        self.assertEqual(10, self.pddl_action_dto.get_duration())
