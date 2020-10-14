import unittest
import coverage
from pddl_dao.pddl_dao_factory.pddl_dao_factory_facory import PDDL_DAO_FactoryFactory
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class Test_PDDL_DAO_Type(unittest.TestCase):

    def setUp(self):
        pddl_dao_factory_facory = PDDL_DAO_FactoryFactory()
        pddl_dao_factory = pddl_dao_factory_facory.create_pddl_dao_factory(
            pddl_dao_factory_facory.pddl_dao_families.MONGOENGINE)

        self.pddl_dao_type = pddl_dao_factory.create_dao_pddl_type()
        self.pddl_dto_type = PddlDtoType("robot")

    def tearDown(self):
        self.pddl_dao_type.delete_all()

    def test_pddl_dao_type_save_true(self):
        result = self.pddl_dao_type._save(self.pddl_dto_type)
        self.assertTrue(result)

    def test_pddl_dao_type_save_false_type_already_exist(self):
        result = self.pddl_dao_type._save(self.pddl_dto_type)
        result = self.pddl_dao_type._save(self.pddl_dto_type)
        self.assertFalse(result)

    def test_pddl_dao_type_get_none(self):
        self.pddl_dto_type = self.pddl_dao_type.get("robot")
        self.assertIsNone(self.pddl_dto_type)

    def test_pddl_dao_type_get(self):
        self.pddl_dao_type.save(self.pddl_dto_type)
        self.pddl_dto_type = self.pddl_dao_type.get("robot")
        self.assertEqual("robot", str(self.pddl_dto_type))

    def test_pddl_dao_type_get_all_0(self):
        pddl_dto_type_list = self.pddl_dao_type.get_all()
        self.assertEqual(0, len(pddl_dto_type_list))

    def test_pddl_dao_type_update_true(self):
        self.pddl_dao_type._save(self.pddl_dto_type)
        result = self.pddl_dao_type._update(self.pddl_dto_type)
        self.assertTrue(result)
        self.pddl_dto_type = self.pddl_dao_type.get("robot")
        self.assertEqual("robot", str(self.pddl_dto_type))

    def test_pddl_dao_type_update_flase(self):
        result = self.pddl_dao_type._update(self.pddl_dto_type)
        self.assertFalse(result)

    def test_pddl_dao_type_save_save_true(self):
        result = self.pddl_dao_type.save(self.pddl_dto_type)
        self.assertTrue(result)

    def test_pddl_dao_type_save_update_true(self):
        self.pddl_dao_type.save(self.pddl_dto_type)
        result = self.pddl_dao_type.save(self.pddl_dto_type)
        self.assertTrue(result)

    def test_pddl_dao_type_delete_false_type_not_exist(self):
        result = self.pddl_dao_type.delete(self.pddl_dto_type)
        self.assertFalse(result)

    def test_pddl_dao_type_delete_true(self):
        self.pddl_dao_type.save(self.pddl_dto_type)
        result = self.pddl_dao_type.delete(self.pddl_dto_type)
        self.assertTrue(result)
        self.pddl_dto_type = self.pddl_dao_type.get("robot")
        self.assertIsNone(self.pddl_dto_type)

    def test_pddl_dao_type_delete_all(self):
        self.pddl_dao_type.save(self.pddl_dto_type)
        result = self.pddl_dao_type.delete_all()
        self.assertTrue(result)
        self.assertEqual(0, len(self.pddl_dao_type.get_all()))


# if __name__ == '__main__':
    #cov = coverage.Coverage()
    # cov.start()
    # suite = unittest.TestLoader().loadTestsFromTestCase(Test_PDDL_DAO_Type)
    # unittest.TextTestRunner(verbosity=2).run(suite)
    # cov.stop()
    # cov.save()
    # cov.html_report()
    # cov.report()
