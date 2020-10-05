import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PDDL_DAO_FactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type
from pddl_dao.pddl_dto.pddl_dto_predicate import PDDL_DTO_Predicate
from pddl_dao.pddl_dto.pddl_dto_action import PDDL_DTO_Action
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object
from pddl_dao.pddl_dto.pddl_dto_condition_efect import PDDL_DTO_ConditionEffect


class Test_PDDL_DAO_Action(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PDDL_DAO_FactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_dao_pddl_type()
        self.pddl_dao_predicate = pddl_dao_factory.create_dao_pddl_predicate()
        self.pddl_dao_action = pddl_dao_factory.create_dao_pddl_action()

        self._robot_type = PDDL_DTO_Type("robot")
        self._wp_type = PDDL_DTO_Type("wp")
        self._robot_at = PDDL_DTO_Predicate(
            "robot_at", [self._robot_type, self._wp_type])

        r = PDDL_DTO_Object(self._robot_type, "r")
        s = PDDL_DTO_Object(self._wp_type, "s")
        d = PDDL_DTO_Object(self._wp_type, "d")

        self._condition_1 = PDDL_DTO_ConditionEffect(PDDL_DTO_ConditionEffect.AT_START,
                                                     self._robot_at,
                                                     [r, s])

        self._effect_1 = PDDL_DTO_ConditionEffect(PDDL_DTO_ConditionEffect.AT_START,
                                                  self._robot_at,
                                                  [r, s],
                                                  is_negative=True)

        self._effect_2 = PDDL_DTO_ConditionEffect(PDDL_DTO_ConditionEffect.AT_END,
                                                  self._robot_at,
                                                  [r, d])

        self.pddl_dto_action = PDDL_DTO_Action(
            "navigation", [r, s, d], [self._condition_1], [self._effect_1, self._effect_2])

    def __save_action(self, pddl_dto_action, save_predicates=True, save_types=True, save_update=False):

        for ele in pddl_dto_action.get_parameters_list():
            if(save_types):
                pddl_type = ele.get_pddl_type()
                self.pddl_dao_type.save_update(pddl_type)

        for ele in pddl_dto_action.get_conditions_list():
            if(save_predicates):
                pddl_predicate = ele.get_pddl_predicate()
                self.pddl_dao_predicate.save_update(pddl_predicate)

        for ele in pddl_dto_action.get_effects_list():
            if(save_predicates):
                predicate = ele.get_pddl_predicate()
                self.pddl_dao_predicate.save_update(predicate)

        if(save_update):
            result = self.pddl_dao_action.save_update(
                pddl_dto_action)
        else:
            result = self.pddl_dao_action.save(pddl_dto_action)

        return result

    def tearDown(self):
        self.pddl_dao_action.delete_all()
        self.pddl_dao_predicate.delete_all()
        self.pddl_dao_type.delete_all()

    def test_pddl_dto_action_str_durative(self):
        self.maxDiff = None
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

    def test_pddl_dto_action_str_durative_no_effects(self):
        self.maxDiff = None
        self.pddl_dto_action.set_effects_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (
\t\t(at start (robot_at r s))
\t)
\t:effect (
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dto_action_str_durative_no_conditions(self):
        self.maxDiff = None
        self.pddl_dto_action.set_conditions_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:duration (= ?duration 10)
\t:condition (
\t)
\t:effect (and
\t\t(at start (not (robot_at r s)))
\t\t(at end (robot_at r d))
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dto_action_str_durative_no_parameters(self):
        self.maxDiff = None
        self.pddl_dto_action.set_conditions_list([])
        self.pddl_dto_action.set_effects_list([])
        self.pddl_dto_action.set_parameters_list([])
        self.assertEqual("""\
(:durative-action navigation
\t:parameters ()
\t:duration (= ?duration 10)
\t:condition (
\t)
\t:effect (
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dto_action_str(self):
        self.maxDiff = None
        self.pddl_dto_action.set_durative(False)
        self.assertEqual("""\
(:action navigation
\t:parameters ( ?r - robot ?s - wp ?d - wp)
\t:precondition (
\t\t(at start (robot_at r s))
\t)
\t:effect (and
\t\t(at start (not (robot_at r s)))
\t\t(at end (robot_at r d))
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dao_action_save_true(self):
        result = self.__save_action(self.pddl_dto_action)
        self.assertTrue(result)

    def test_pddl_dao_action_save_false_predicates_not_exist(self):
        result = self.__save_action(
            self.pddl_dto_action, save_predicates=False)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_types_not_exist(self):
        result = self.__save_action(
            self.pddl_dto_action, save_types=False)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_incorrect_condition_types(self):
        self.pddl_dto_action.get_conditions_list(
        )[0].get_pddl_objects_list().reverse()
        result = self.__save_action(
            self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_incorrect_condition_len(self):
        self.pddl_dto_action.get_conditions_list(
        )[0].set_pddl_objects_list([])
        result = self.__save_action(
            self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_false_action_already_exist(self):
        result = self.__save_action(self.pddl_dto_action)
        result = self.__save_action(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_get_none(self):
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)

    def test_pddl_dao_action_get(self):
        self.__save_action(self.pddl_dto_action)

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
        self.__save_action(self.pddl_dto_action)
        result = self.pddl_dao_action.update(self.pddl_dto_action)
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
\t\t(at start (not (robot_at r s)))
\t\t(at end (robot_at r d))
\t)
)""",
                         str(self.pddl_dto_action))

    def test_pddl_dao_action_update_flase(self):
        result = self.pddl_dao_action.update(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_save_update_save_true(self):
        result = self.__save_action(
            self.pddl_dto_action, save_update=True)
        self.assertTrue(result)

    def test_pddl_dao_action_save_update_update_true(self):
        result = self.__save_action(self.pddl_dto_action)
        result = self.__save_action(
            self.pddl_dto_action, save_update=True)
        self.assertTrue(result)

    def test_pddl_dao_action_delete_false_action_not_exist(self):
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_delete_false_predicates_not_exist(self):
        self.__save_action(
            self.pddl_dto_action, save_predicates=False)
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_delete_false_typess_not_exist(self):
        self.__save_action(
            self.pddl_dto_action, save_types=False)
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertFalse(result)

    def test_pddl_dao_action_delete_true(self):
        self.__save_action(self.pddl_dto_action)
        result = self.pddl_dao_action.delete(self.pddl_dto_action)
        self.assertTrue(result)
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)

    def test_pddl_dao_action_delete_all(self):
        self.__save_action(self.pddl_dto_action)
        result = self.pddl_dao_action.delete_all()
        self.assertTrue(result)
        self.pddl_dto_action = self.pddl_dao_action.get("navigation")
        self.assertIsNone(self.pddl_dto_action)
