import unittest
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto


class Test_PDDL_DAO_Predicate(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()

        robot_type = PddlTypeDto("robot")
        wp_type = PddlTypeDto("wp")
        self.pddl_predicate_dto = PddlPredicateDto(
            "robot_at", [robot_type, wp_type])

    def tearDown(self):
        self.pddl_predicate_dao.delete_all()
        self.pddl_type_dao.delete_all()

    def test_pddl_dao_predicate_save_true(self):
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_dao_predicate_save_false_predicate_already_exist(self):
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_dao_predicate_get_none(self):
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertIsNone(self.pddl_predicate_dto)

    def test_pddl_dao_predicate_get(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_predicate_dto))

    def test_pddl_dao_predicate_get_all_0(self):
        pddl_dto_predicate_list = self.pddl_predicate_dao.get_all()
        self.assertEqual(0, len(pddl_dto_predicate_list))

    def test_pddl_dao_predicate_update_true(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao._update(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_predicate_dto))

    def test_pddl_dao_predicate_update_flase(self):
        result = self.pddl_predicate_dao._update(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_dao_predicate_save_save_true(self):
        result = self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_dao_predicate_save_update_true(self):
        self.pddl_predicate_dao._save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        self.assertTrue(result)

    def test_pddl_dao_predicate_delete_false_predicate_not_exist(self):
        result = self.pddl_predicate_dao.delete(self.pddl_predicate_dto)
        self.assertFalse(result)

    def test_pddl_dao_predicate_delete_true(self):
        self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.delete(self.pddl_predicate_dto)
        self.assertTrue(result)
        self.pddl_predicate_dto = self.pddl_predicate_dao.get("robot_at")
        self.assertIsNone(self.pddl_predicate_dto)

    def test_pddl_dao_predicate_delete_all(self):
        self.pddl_predicate_dao.save(self.pddl_predicate_dto)
        result = self.pddl_predicate_dao.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_predicate_dao.get_all()))
