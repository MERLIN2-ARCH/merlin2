import unittest
import coverage
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type
from pddl_dao.pddl_dto.pddl_dto_predicate import PDDL_DTO_Predicate


class Test_PDDL_DTO_Predicate(unittest.TestCase):

    def setUp(self):

        robot_type = PDDL_DTO_Type("robot")
        wp_type = PDDL_DTO_Type("wp")
        self.pddl_dto_predicate = PDDL_DTO_Predicate(
            "robot_at", [robot_type, wp_type])

    def test_pddl_dto_predicate_str(self):
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_dto_predicate))

    def test_pddl_dto_predicate_str_no_types(self):
        self.pddl_dto_predicate.set_pddl_types_list(None)
        self.assertEqual("(robot_at)",
                         str(self.pddl_dto_predicate))

    def test_pddl_dto_predicate_get_predicate_name(self):
        self.assertEqual("robot_at",
                         self.pddl_dto_predicate.get_predicate_name())

    def test_pddl_dto_predicate_get_pddl_types(self):
        pddl_types_list = self.pddl_dto_predicate.get_pddl_types_list()
        self.assertEqual("robot", pddl_types_list[0].get_type_name())
        self.assertEqual("wp", pddl_types_list[1].get_type_name())
