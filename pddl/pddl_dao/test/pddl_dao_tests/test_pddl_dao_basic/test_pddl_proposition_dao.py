import unittest
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dto.pddl_object_dto import PddlObjectDto


class TestPddlPropositionDao(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_proposition_dao = pddl_dao_factory.create_pddl_proposition_dao()

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto(
            "robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlObjectDto(self._robot_type, "rb1")
        self._wp1 = PddlObjectDto(self._wp_type, "wp1")
        self.pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._rb1, self._wp1])

    def tearDown(self):
        self.pddl_object_dao.delete_all()
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()
        self.pddl_proposition_dao.delete_all()

    def test_pddl_dao_proposition_save_true(self):
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_dao_proposition_save_false_incorrect_proposition_types(self):
        self.pddl_proposition_dto.get_pddl_objects_list().reverse()
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_false_incorrect_proposition_len(self):
        self.pddl_proposition_dto.set_pddl_objects_list([])
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_false_proposition_already_exist(self):
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_get_by_predicate_none(self):
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate(
            "robot_at")
        self.assertIsNone(self.pddl_proposition_dto)

    def test_pddl_dao_proposition_get_by_predicate(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)

        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate("robot_at")[
            0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_get_goals(self):
        self.pddl_proposition_dto.set_is_goal(True)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_get_goals_empty(self):
        self.pddl_proposition_dto.set_is_goal(False)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_goals()
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_get_no_goals(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_no_goals()[0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_get_no_goals_empty(self):
        self.pddl_proposition_dto.set_is_goal(True)
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_no_goals()
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_get_all_0(self):
        pddl_dto_proposition_list = self.pddl_proposition_dao.get_all()
        self.assertEqual(0, len(pddl_dto_proposition_list))

    def test_pddl_dao_proposition_update_true(self):
        self.pddl_proposition_dao._save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate("robot_at")[
            0]
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_update_flase_proposition_not_exists(self):
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_update_false_incorrect_proposition_types(self):
        self.pddl_proposition_dto.get_pddl_objects_list().reverse()
        result = self.pddl_proposition_dao._update(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_save_save_true(self):
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_dao_proposition_save_update_true(self):
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        self.assertTrue(result)

    def test_pddl_dao_proposition_delete_false_proposition_not_exist(self):
        result = self.pddl_proposition_dao.delete(self.pddl_proposition_dto)
        self.assertFalse(result)

    def test_pddl_dao_proposition_delete_true(self):
        self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.delete(self.pddl_proposition_dto)
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate(
            "robot_at")
        self.assertEqual(0, len(self.pddl_proposition_dto))

    def test_pddl_dao_proposition_delete_all(self):
        self.pddl_proposition_dao.save(self.pddl_proposition_dto)
        result = self.pddl_proposition_dao.delete_all()
        self.assertTrue(result)
        self.pddl_proposition_dto = self.pddl_proposition_dao.get_by_predicate(
            "robot_at")
        self.assertEqual(0, len(self.pddl_proposition_dto))
