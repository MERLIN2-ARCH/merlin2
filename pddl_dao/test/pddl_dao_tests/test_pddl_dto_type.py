import unittest
import coverage
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class Test_PDDL_DAO_Type(unittest.TestCase):

    def setUp(self):

        self.pddl_dto_type = PddlDtoType("robot")

    def test_pddl_dto_type_str(self):
        self.assertEqual("robot", str(self.pddl_dto_type))

    def test_pddl_dto_type_get_type_name(self):
        self.assertEqual("robot", self.pddl_dto_type.get_type_name())
