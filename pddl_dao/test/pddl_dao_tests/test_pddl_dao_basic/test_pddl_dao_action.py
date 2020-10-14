import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PDDL_DAO_FactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate
from pddl_dao.pddl_dto.pddl_dto_action import PddlDtoAction
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject
from pddl_dao.pddl_dto.pddl_dto_condition_efect import PddlDtoConditionEffect


class Test_PDDL_DAO_Action(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PDDL_DAO_FactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_dao_pddl_type()
        self.pddl_dao_predicate = pddl_dao_factory.create_dao_pddl_predicate()
        self.pddl_dao_action = pddl_dao_factory.create_dao_pddl_action()

        self._robot_type = PddlDtoType("robot")
        self._wp_type = PddlDtoType("wp")
        self._robot_at = PddlDtoPredicate(
            "robot_at", [self._robot_type, self._wp_type])

        r = PddlDtoObject(self._robot_type, "r")
        s = PddlDtoObject(self._wp_type, "s")
        d = PddlDtoObject(self._wp_type, "d")

        self._condition_1 = PddlDtoConditionEffect(PddlDtoConditionEffect.AT_START,
                                                   self._robot_at,
                                                   [r, s])

        self._effect_1 = PddlDtoConditionEffect(PddlDtoConditionEffect.AT_START,
                                                self._robot_at,
                                                [r, s],
                                                is_negative=True)

        self._effect_2 = PddlDtoConditionEffect(PddlDtoConditionEffect.AT_END,
                                                self._robot_at,
                                                [r, d])

        self.pddl_dto_action = PddlDtoAction(
            "navigation", [r, s, d], [self._condition_1], [self._effect_1, self._effect_2])

    def tearDown(self):
        self.pddl_dao_action.delete_all()
        self.pddl_dao_predicate.delete_all()
        self.pddl_dao_type.delete_all()

    def test_pddl_dao_action_save_true(self):
        result = self.pddl_dao_action._save(self.pddl_dto_action)
        self.assertTrue(result)

    def test_pddl_dao_action_save_false_incorrect_condition_types(self):
        self.pddl_dto_action.get_conditions_list(
        )[0].get_pddl_objects_list().reverse()
        result = self.pddl_dao_action._save(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_incorrect_condition_len(self):
        self.pddl_dto_action.get_conditions_list(
        )[0].set_pddl_objects_list([])
        result = self.pddl_dao_action._save(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_action_already_exist(self):
        result = self.pddl_dao_action._save(self.pddl_dto_action)
        result = self.pddl_dao_action._save(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_get_none(self):
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)

    def test_pddl_dao_action_get(self):
        self.pddl_dao_action._save(self.pddl_dto_action)

        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (
\t\t(at start (robot_at r s))
\t)
\t:effect (and
\t\t(at start (not (robot_at r s)))
\t\t(at end (robot_at r d))
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dao_action_get_all_0(self):
        pddl_dto_action_list = self.pddl_dao_action.get_all()
        self.assertEqual(0, len(pddl_dto_action_list))

    def test_pddl_dao_action_update_true(self):
        self.pddl_dao_action._save(self.pddl_dto_action)
        self.pddl_dto_action.get_effects_list()[0].set_time(
            PddlDtoConditionEffect.AT_END)
        result = self.pddl_dao_action._update(self.pddl_dto_action)
        self.assertTrue(result)
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (
\t\t(at start (robot_at r s))
\t)
\t:effect (and
\t\t(at end (not (robot_at r s)))
\t\t(at end (robot_at r d))
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dao_action_update_flase(self):
        result = self.pddl_dao_action._update(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_save_true(self):
        result = self.pddl_dao_action.save(self.pddl_dto_action)
        self.assertTrue(result)

    def test_pddl_dao_action_save_update_true(self):
        result = self.pddl_dao_action.save(self.pddl_dto_action)
        result = self.pddl_dao_action.save(self.pddl_dto_action)
        self.assertTrue(result)

    def test_pddl_dao_action_delete_false_action_not_exist(self):
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_delete_true(self):
        self.pddl_dao_action.save(self.pddl_dto_action)
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertTrue(result)
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)

    def test_pddl_dao_action_delete_all(self):
        self.pddl_dao_action.save(self.pddl_dto_action)
        result = self.pddl_dao_action.delete_all()
        self.assertTrue(result)
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)
