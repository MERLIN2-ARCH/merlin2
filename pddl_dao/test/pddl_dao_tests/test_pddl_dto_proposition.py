import unittest
import coverage
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate
from pddl_dao.pddl_dto.pddl_dto_proposition import PddlDtoProposition
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class Test_PDDL_DTO_Proposition(unittest.TestCase):

    def setUp(self):

        self._robot_type = PddlDtoType("robot")
        self._wp_type = PddlDtoType("wp")
        self._robot_at = PddlDtoPredicate(
            "robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlDtoObject(self._robot_type, "rb1")
        self._wp1 = PddlDtoObject(self._wp_type, "wp1")
        self.pddl_dto_proposition = PddlDtoProposition(
            self._robot_at, [self._rb1, self._wp1])

    def test_pddl_dto_proposition_str(self):
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_dto_proposition))

    def test_pddl_dto_proposition_str_no_objects(self):
        self.pddl_dto_proposition.set_pddl_objects_list([])
        self.assertEqual("(robot_at)",
                         str(self.pddl_dto_proposition))

    def test_pddl_dto_proposition_get_pddl_predicate(self):
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_dto_proposition.get_pddl_predicate()))

    def test_pddl_dto_proposition_get_pddl_objects(self):
        pddl_objects_list = self.pddl_dto_proposition.get_pddl_objects_list()
        self.assertEqual("rb1",                         str(
            pddl_objects_list[0].get_object_name()))
        self.assertEqual("wp1",                         str(
            pddl_objects_list[1].get_object_name()))

    def test_pddl_dto_proposition_get_is_goal(self):
        self.assertFalse(self.pddl_dto_proposition.get_is_goal())
