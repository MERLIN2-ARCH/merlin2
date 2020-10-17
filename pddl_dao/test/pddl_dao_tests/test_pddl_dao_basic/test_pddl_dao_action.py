import unittest
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto


class TestPddlDaoAction(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()

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

    def tearDown(self):
        self.pddl_action_dao.delete_all()
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()

    def test_pddl_dao_action_save_true(self):
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_dao_action_save_false_incorrect_condition_types(self):
        self.pddl_action_dto.get_conditions_list(
        )[0].get_pddl_objects_list().reverse()
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_incorrect_condition_len(self):
        self.pddl_action_dto.get_conditions_list(
        )[0].set_pddl_objects_list([])
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_action_already_exist(self):
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        result = self.pddl_action_dao._save(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_dao_action_get_none(self):
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)

    def test_pddl_dao_action_get(self):
        self.pddl_action_dao._save(self.pddl_action_dto)

        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
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

    def test_pddl_dao_action_get_all_0(self):
        pddl_dto_action_list = self.pddl_action_dao.get_all()
        self.assertEqual(0, len(pddl_dto_action_list))

    def test_pddl_dao_action_update_true(self):
        self.pddl_action_dao._save(self.pddl_action_dto)
        self.pddl_action_dto.get_effects_list()[0].set_time(
            PddlConditionEffectDto.AT_END)
        result = self.pddl_action_dao._update(self.pddl_action_dto)
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (and
\t\t(at start (robot_at ?r ?s))
\t)
\t:effect (and
\t\t(at end (not (robot_at ?r ?s)))
\t\t(at end (robot_at ?r ?d))
\t)
)""",
                         str(self.pddl_action_dto))

    def test_pddl_dao_action_update_flase(self):
        result = self.pddl_action_dao._update(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_dao_action_save_save_true(self):
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_dao_action_save_update_true(self):
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.save(self.pddl_action_dto)
        self.assertTrue(result)

    def test_pddl_dao_action_delete_false_action_not_exist(self):
        result = self.pddl_action_dao.delete(self.pddl_action_dto)
        self.assertFalse(result)

    def test_pddl_dao_action_delete_true(self):
        self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.delete(self.pddl_action_dto)
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)

    def test_pddl_dao_action_delete_all(self):
        self.pddl_action_dao.save(self.pddl_action_dto)
        result = self.pddl_action_dao.delete_all()
        self.assertTrue(result)
        self.pddl_action_dto = self.pddl_action_dao.get("navigation")
        self.assertIsNone(self.pddl_action_dto)
