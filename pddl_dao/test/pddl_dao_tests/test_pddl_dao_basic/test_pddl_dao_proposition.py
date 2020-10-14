import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate
from pddl_dao.pddl_dto.pddl_dto_proposition import PddlDtoProposition
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class Test_PDDL_DAO_Proposition(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_pddl_dao_type()
        self.pddl_dao_object = pddl_dao_factory.create_pddl_dao_object()
        self.pddl_dao_predicate = pddl_dao_factory.create_pddl_dao_predicate()
        self.pddl_dao_proposition = pddl_dao_factory.create_pddl_dao_proposition()

        self._robot_type = PddlDtoType("robot")
        self._wp_type = PddlDtoType("wp")
        self._robot_at = PddlDtoPredicate(
            "robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlDtoObject(self._robot_type, "rb1")
        self._wp1 = PddlDtoObject(self._wp_type, "wp1")
        self.pddl_dto_proposition = PddlDtoProposition(
            self._robot_at, [self._rb1, self._wp1])

    def tearDown(self):
        self.pddl_dao_object.delete_all()
        self.pddl_dao_predicate.delete_all()
        self.pddl_dao_type.delete_all()
        self.pddl_dao_proposition.delete_all()

    def test_pddl_dao_proposition_save_true(self):
        result = self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        self.assertTrue(result)

    def test_pddl_dao_proposition_save_false_incorrect_proposition_types(self):
        self.pddl_dto_proposition.get_pddl_objects_list().reverse()
        result = self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_false_incorrect_proposition_len(self):
        self.pddl_dto_proposition.set_pddl_objects_list([])
        result = self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_false_proposition_already_exist(self):
        result = self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        result = self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_get_by_predicate_none(self):
        self.pddl_dto_proposition = self.pddl_dao_proposition.get_by_predicate(
            "robot_at")
        self.assertIsNone(self.pddl_dto_proposition)

    def test_pddl_dao_proposition_get_by_predicate(self):
        self.pddl_dao_proposition._save(self.pddl_dto_proposition)

        self.pddl_dto_proposition = self.pddl_dao_proposition.get_by_predicate("robot_at")[
            0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_dto_proposition))

    def test_pddl_dao_proposition_get_goals(self):
        self.pddl_dto_proposition.set_is_goal(True)
        self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        self.pddl_dto_proposition = self.pddl_dao_proposition.get_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_dto_proposition))

    def test_pddl_dao_proposition_get_all_0(self):
        pddl_dto_proposition_list = self.pddl_dao_proposition.get_all()
        self.assertEqual(0, len(pddl_dto_proposition_list))

    def test_pddl_dao_proposition_update_true(self):
        self.pddl_dao_proposition._save(self.pddl_dto_proposition)
        result = self.pddl_dao_proposition._update(self.pddl_dto_proposition)
        self.assertTrue(result)
        self.pddl_dto_proposition = self.pddl_dao_proposition.get_by_predicate("robot_at")[
            0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_dto_proposition))

    def test_pddl_dao_proposition_update_flase_proposition_not_exists(self):
        result = self.pddl_dao_proposition._update(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_update_false_incorrect_proposition_types(self):
        self.pddl_dto_proposition.get_pddl_objects_list().reverse()
        result = self.pddl_dao_proposition._update(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_save_true(self):
        result = self.pddl_dao_proposition.save(self.pddl_dto_proposition)
        self.assertTrue(result)

    def test_pddl_dao_proposition_save_update_true(self):
        result = self.pddl_dao_proposition.save(self.pddl_dto_proposition)
        result = self.pddl_dao_proposition.save(self.pddl_dto_proposition)
        self.assertTrue(result)

    def test_pddl_dao_proposition_delete_false_proposition_not_exist(self):
        result = self.pddl_dao_proposition.delete(self.pddl_dto_proposition)
        self.assertFalse(result)

    def test_pddl_dao_proposition_delete_true(self):
        self.pddl_dao_proposition.save(self.pddl_dto_proposition)
        result = self.pddl_dao_proposition.delete(self.pddl_dto_proposition)
        self.assertTrue(result)
        self.pddl_dto_proposition = self.pddl_dao_proposition.get_by_predicate(
            "robot_at")
        self.assertEqual(0, len(self.pddl_dto_proposition))

    def test_pddl_dao_proposition_delete_all(self):
        self.pddl_dao_proposition.save(self.pddl_dto_proposition)
        result = self.pddl_dao_proposition.delete_all()
        self.assertTrue(result)
        self.pddl_dto_proposition = self.pddl_dao_proposition.get_by_predicate(
            "robot_at")
        self.assertEqual(0, len(self.pddl_dto_proposition))
