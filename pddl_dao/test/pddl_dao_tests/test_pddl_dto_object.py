import unittest
import coverage
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class Test_PDDL_DTO_Object(unittest.TestCase):

    def setUp(self):

        pddl_dto_type = PddlDtoType("robot")
        self.pddl_dto_object = PddlDtoObject(pddl_dto_type, "rb1")

    def test_pddl_dto_object_str(self):
        self.assertEqual("rb1 - robot", str(self.pddl_dto_object))

    def test_pddl_dto_type_get_pddl_type(self):
        self.assertEqual("robot", str(self.pddl_dto_object.get_pddl_type()))

    def test_pddl_dto_type_get_object_name(self):
        self.assertEqual("rb1", self.pddl_dto_object.get_object_name())
