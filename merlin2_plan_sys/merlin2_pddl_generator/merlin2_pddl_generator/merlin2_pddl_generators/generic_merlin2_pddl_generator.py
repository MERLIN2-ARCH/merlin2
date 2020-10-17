
""" Merlin2 Pddl Generator Generic"""

from typing import List

from merlin2_pddl_generator.merlin2_pddl_parser.merlin2_pddl_domain_parser import (
    Merlin2PddlDomainParser
)
from merlin2_pddl_generator.merlin2_pddl_parser.merlin2_pddl_problem_parser import (
    Merlin2PddlProblemParser
)

from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class GenericMerlin2PddlGenerator:
    """ Merlin2 Pddl Generator Class"""

    def __init__(self, pddl_dao_factory: PddlDaoFactory):
        self.domain_parser = Merlin2PddlDomainParser()
        self.problem_parser = Merlin2PddlProblemParser()

        self.pddl_type_dao = pddl_dao_factory.create_pddl_type_dao()
        self.pddl_predicate_dao = pddl_dao_factory.create_pddl_predicate_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()
        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()
        self.pddl_proposition_dao = pddl_dao_factory.create_pddl_proposition_dao()

    def get_pddl_dtos(self):
        """ gets all dtos and returns them in a list

        Returns:
            List[PddlDto]: list of dtos:
                                0: types
                                1: predicates
                                2: actions
                                3: objects
                                4: propositions
                                5: goals
        """

        pddl_type_dtos = self.pddl_type_dao.get_all()
        pddl_predicate_dtos = self.pddl_predicate_dao.get_all()
        pddl_action_dtos = self.pddl_action_dao.get_all()
        pddl_object_dtos = self.pddl_object_dao.get_all()
        pddl_proposition_dtos = self.pddl_proposition_dao.get_no_goals()
        pddl_goal_dtos = self.pddl_proposition_dao.get_goals()

        return [pddl_type_dtos, pddl_predicate_dtos, pddl_action_dtos,
                pddl_object_dtos, pddl_proposition_dtos, pddl_goal_dtos]

    def generate_pddl(self) -> List[PddlDto]:
        """ generate pddl domain and problem

        Returns:
            List[PddlDto]: list of two elements,
                       the first element is the domain, the second the problem
        """

        dto_list = self.get_pddl_dtos()

        types_list = dto_list[0]
        predicates_list = dto_list[1]
        actions_list = dto_list[2]
        objects_list = dto_list[3]
        propositions_list = dto_list[4]
        goals_list = dto_list[5]

        pddl_domain = self.domain_parser.parse_pddl_domain_dto(types_list,
                                                               predicates_list,
                                                               actions_list)

        pddl_problem = self.problem_parser.parse_pddl_problem_dto(objects_list,
                                                                  propositions_list,
                                                                  goals_list)

        return [pddl_domain, pddl_problem]
