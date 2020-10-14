import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate


class Test_PDDL_DAO_Predicate(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_pddl_dao_type()
        self.pddl_dao_predicate = pddl_dao_factory.create_pddl_dao_predicate()

        robot_type = PddlDtoType("robot")
        wp_type = PddlDtoType("wp")
        self.pddl_dto_predicate = PddlDtoPredicate(
            "robot_at", [robot_type, wp_type])

    def tearDown(self):
        self.pddl_dao_predicate.delete_all()
        self.pddl_dao_type.delete_all()

    def test_pddl_dao_predicate_save_true(self):
        result = self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        self.assertTrue(result)

    def test_pddl_dao_predicate_save_false_predicate_already_exist(self):
        result = self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        result = self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        self.assertFalse(result)

    def test_pddl_dao_predicate_get_none(self):
        self.pddl_dto_predicate = self.pddl_dao_predicate.get("robot_at")
        self.assertIsNone(self.pddl_dto_predicate)

    def test_pddl_dao_predicate_get(self):
        self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        self.pddl_dto_predicate = self.pddl_dao_predicate.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_dto_predicate))

    def test_pddl_dao_predicate_get_all_0(self):
        pddl_dto_predicate_list = self.pddl_dao_predicate.get_all()
        self.assertEqual(0, len(pddl_dto_predicate_list))

    def test_pddl_dao_predicate_update_true(self):
        self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        result = self.pddl_dao_predicate._update(self.pddl_dto_predicate)
        self.assertTrue(result)
        self.pddl_dto_predicate = self.pddl_dao_predicate.get("robot_at")
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_dto_predicate))

    def test_pddl_dao_predicate_update_flase(self):
        result = self.pddl_dao_predicate._update(self.pddl_dto_predicate)
        self.assertFalse(result)

    def test_pddl_dao_predicate_save_save_true(self):
        result = self.pddl_dao_predicate.save(self.pddl_dto_predicate)
        self.assertTrue(result)

    def test_pddl_dao_predicate_save_update_true(self):
        self.pddl_dao_predicate._save(self.pddl_dto_predicate)
        result = self.pddl_dao_predicate.save(self.pddl_dto_predicate)
        self.assertTrue(result)

    def test_pddl_dao_predicate_delete_false_predicate_not_exist(self):
        result = self.pddl_dao_predicate.delete(self.pddl_dto_predicate)
        self.assertFalse(result)

    def test_pddl_dao_predicate_delete_true(self):
        self.pddl_dao_predicate.save(self.pddl_dto_predicate)
        result = self.pddl_dao_predicate.delete(self.pddl_dto_predicate)
        self.assertTrue(result)
        self.pddl_dto_predicate = self.pddl_dao_predicate.get("robot_at")
        self.assertIsNone(self.pddl_dto_predicate)

    def test_pddl_dao_predicate_delete_all(self):
        self.pddl_dao_predicate.save(self.pddl_dto_predicate)
        result = self.pddl_dao_predicate.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_dao_predicate.get_all()))
