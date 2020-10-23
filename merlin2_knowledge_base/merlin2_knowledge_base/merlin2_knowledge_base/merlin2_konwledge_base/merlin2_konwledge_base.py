
""" Merlin2 Knowledge Base """

from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto


class Merlin2KnowledgeBase:
    """ Merlin2 Knowledge Base Class """

    def __init__(self):
        self.types_dict = {}
        self.objects_dict = {}
        self.predicates_dict = {}
        self.propositions_list = {}
        self.actions_dict = {}

    def get_type(self, type_name: str) -> PddlTypeDto:
        """ get the PddlTypeDto with a given type name

        Args:
            type_name (str): type name

        Returns:
            PddlTypeDto: PddlTypeDto
        """

        if type_name in self.types_dict:
            return self.types_dict[type_name]

        return None

    def save_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save a new pddl type
            if alrready exists, returns false

        Args:
            type (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        if pddl_type_dto.get_type_name() in self.types_dict:
            return False

        self.types_dict[pddl_type_dto.get_type_name()] = pddl_type_dto

        return True

    def update_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ update a pddl type
            if not exists, returns false

        Args:
            type (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        if not pddl_type_dto.get_type_name() in self.types_dict:
            return False

        self.types_dict[pddl_type_dto.get_type_name()] = pddl_type_dto

        return True

    def delete_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ delete a pddl type
            if not exists, returns false

        Args:
            type (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        if not pddl_type_dto.get_type_name() in self.types_dict:
            return False

        del self.types_dict[pddl_type_dto.get_type_name()]

        return True

    def delete_all_type(self) -> bool:
        """ delete a all pddl type

        Returns:
            bool: succeed
        """

        self.types_dict = {}

        return True
