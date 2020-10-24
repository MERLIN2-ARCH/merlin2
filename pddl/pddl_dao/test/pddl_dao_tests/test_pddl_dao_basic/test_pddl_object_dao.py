import unittest
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PddlDaoFactoryFactory
from pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dto.pddl_object_dto import PddlObjectDto


class TestPddlObjectDao(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PddlDaoFactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()

        pddl_type_dto = PddlTypeDto("robot")
        self.pddl_object_dto = PddlObjectDto(pddl_type_dto, "rb1")

    def tearDown(self):
        self.pddl_object_dao.delete_all()

    def test_pddl_dao_object_save_true(self):
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_dao_object_save_false_object_already_exist(self):
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao._save(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_dao_object_get_none(self):
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertIsNone(self.pddl_object_dto)

    def test_pddl_dao_object_get(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_dao_object_get_all_0(self):
        pddl_dto_object_list = self.pddl_object_dao.get_all()
        self.assertEqual(0, len(pddl_dto_object_list))

    def test_pddl_dao_object_update_true(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao._update(self.pddl_object_dto)
        self.assertTrue(result)
        self.pddl_object_dto = self.pddl_object_dao.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_dao_object_update_flase(self):
        result = self.pddl_object_dao._update(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_dao_object_save_save_true(self):
        result = self.pddl_object_dao.save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_dao_object_save_true(self):
        self.pddl_object_dao._save(self.pddl_object_dto)
        result = self.pddl_object_dao.save(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_dao_object_delete_false_object_not_exist(self):
        result = self.pddl_object_dao.delete(self.pddl_object_dto)
        self.assertFalse(result)

    def test_pddl_dao_object_delete_true(self):
        self.pddl_object_dao.save(self.pddl_object_dto)
        result = self.pddl_object_dao.delete(self.pddl_object_dto)
        self.assertTrue(result)

    def test_pddl_dao_object_delete_all(self):
        self.pddl_object_dao.save(self.pddl_object_dto)
        result = self.pddl_object_dao.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_object_dao.get_all()))
