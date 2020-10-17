import unittest
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto


class TestPddlDtoProposition(unittest.TestCase):

    def setUp(self):

        self._robot_type = PddlTypeDto("robot")
        self._wp_type = PddlTypeDto("wp")
        self._robot_at = PddlPredicateDto(
            "robot_at", [self._robot_type, self._wp_type])

        self._rb1 = PddlObjectDto(self._robot_type, "rb1")
        self._wp1 = PddlObjectDto(self._wp_type, "wp1")
        self.pddl_proposition_dto = PddlPropositionDto(
            self._robot_at, [self._rb1, self._wp1])

    def test_pddl_dto_proposition_str(self):
        self.assertEqual("(robot_at rb1 wp1)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dto_proposition_str_no_objects(self):
        self.pddl_proposition_dto.set_pddl_objects_list([])
        self.assertEqual("(robot_at)",
                         str(self.pddl_proposition_dto))

    def test_pddl_dto_proposition_get_pddl_predicate(self):
        self.assertEqual("(robot_at ?r0 - robot ?w1 - wp)",
                         str(self.pddl_proposition_dto.get_pddl_predicate()))

    def test_pddl_dto_proposition_get_pddl_objects(self):
        pddl_objects_list = self.pddl_proposition_dto.get_pddl_objects_list()
        self.assertEqual("rb1",                         str(
            pddl_objects_list[0].get_object_name()))
        self.assertEqual("wp1",                         str(
            pddl_objects_list[1].get_object_name()))

    def test_pddl_dto_proposition_get_is_goal(self):
        self.assertFalse(self.pddl_proposition_dto.get_is_goal())
