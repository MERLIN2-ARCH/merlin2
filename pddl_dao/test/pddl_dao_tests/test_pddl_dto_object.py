import unittest
import coverage
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object


class Test_PDDL_DTO_Object(unittest.TestCase):

    def setUp(self):

        pddl_dto_type = PDDL_DTO_Type("robot")
        self.pddl_dto_object = PDDL_DTO_Object(pddl_dto_type, "rb1")

    def test_pddl_dto_object_str(self):
        self.assertEqual("rb1 - robot", str(self.pddl_dto_object))

    def test_pddl_dto_type_get_pddl_type(self):
        self.assertEqual("robot", str(self.pddl_dto_object.get_pddl_type()))

    def test_pddl_dto_type_get_object_name(self):
        self.assertEqual("rb1", self.pddl_dto_object.get_object_name())
