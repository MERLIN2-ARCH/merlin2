import unittest
import coverage
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto


class Test_PDDL_DTO_Object(unittest.TestCase):

    def setUp(self):

        pddl_type_dto = PddlTypeDto("robot")
        self.pddl_object_dto = PddlObjectDto(pddl_type_dto, "rb1")

    def test_pddl_dto_object_str(self):
        self.assertEqual("rb1 - robot", str(self.pddl_object_dto))

    def test_pddl_dto_type_get_pddl_type(self):
        self.assertEqual("robot", str(self.pddl_object_dto.get_pddl_type()))

    def test_pddl_dto_type_get_object_name(self):
        self.assertEqual("rb1", self.pddl_object_dto.get_object_name())
