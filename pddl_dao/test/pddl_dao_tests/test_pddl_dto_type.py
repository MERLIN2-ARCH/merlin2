import unittest
import coverage
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto


class Test_PDDL_DAO_Type(unittest.TestCase):

    def setUp(self):

        self.pddl_type_dto = PddlTypeDto("robot")

    def test_pddl_dto_type_str(self):
        self.assertEqual("robot", str(self.pddl_type_dto))

    def test_pddl_dto_type_get_type_name(self):
        self.assertEqual("robot", self.pddl_type_dto.get_type_name())
