import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PDDL_DAO_FactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class Test_PDDL_DAO_Object(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PDDL_DAO_FactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_object = pddl_dao_factory.create_dao_pddl_object()

        pddl_dto_type = PddlDtoType("robot")
        self.pddl_dto_object = PddlDtoObject(pddl_dto_type, "rb1")

    def tearDown(self):
        self.pddl_dao_object.delete_all()

    def test_pddl_dao_object_save_true(self):
        result = self.pddl_dao_object._save(self.pddl_dto_object)
        self.assertTrue(result)

    def test_pddl_dao_object_save_false_object_already_exist(self):
        result = self.pddl_dao_object._save(self.pddl_dto_object)
        result = self.pddl_dao_object._save(self.pddl_dto_object)
        self.assertFalse(result)

    def test_pddl_dao_object_get_none(self):
        self.pddl_dto_object = self.pddl_dao_object.get("rb1")
        self.assertIsNone(self.pddl_dto_object)

    def test_pddl_dao_object_get(self):
        self.pddl_dao_object._save(self.pddl_dto_object)
        self.pddl_dto_object = self.pddl_dao_object.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_dto_object))

    def test_pddl_dao_object_get_all_0(self):
        pddl_dto_object_list = self.pddl_dao_object.get_all()
        self.assertEqual(0, len(pddl_dto_object_list))

    def test_pddl_dao_object_update_true(self):
        self.pddl_dao_object._save(self.pddl_dto_object)
        result = self.pddl_dao_object._update(self.pddl_dto_object)
        self.assertTrue(result)
        self.pddl_dto_object = self.pddl_dao_object.get("rb1")
        self.assertEqual("rb1 - robot", str(self.pddl_dto_object))

    def test_pddl_dao_object_update_flase(self):
        result = self.pddl_dao_object._update(self.pddl_dto_object)
        self.assertFalse(result)

    def test_pddl_dao_object_save_save_true(self):
        result = self.pddl_dao_object.save(self.pddl_dto_object)
        self.assertTrue(result)

    def test_pddl_dao_object_save_true(self):
        self.pddl_dao_object._save(self.pddl_dto_object)
        result = self.pddl_dao_object.save(self.pddl_dto_object)
        self.assertTrue(result)

    def test_pddl_dao_object_delete_false_object_not_exist(self):
        result = self.pddl_dao_object.delete(self.pddl_dto_object)
        self.assertFalse(result)

    def test_pddl_dao_object_delete_true(self):
        self.pddl_dao_object.save(self.pddl_dto_object)
        result = self.pddl_dao_object.delete(self.pddl_dto_object)
        self.assertTrue(result)

    def test_pddl_dao_object_delete_all(self):
        self.pddl_dao_object.save(self.pddl_dto_object)
        result = self.pddl_dao_object.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_dao_object.get_all()))
