
""" Merlin2 Knowledge Base """
from typing import List

from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto


class Merlin2KnowledgeBase:
    """ Merlin2 Knowledge Base Class """

    def __init__(self):
        self.types_dict = {}
        self.objects_dict = {}
        self.predicates_dict = {}
        self.propositions_list = []
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

    def get_all_types(self) -> List[PddlTypeDto]:
        """ get all PddlTypeDto

        Returns:
            List[PddlTypeDto]: list of PddlTypeDto
        """

        type_list = []

        for type_name in self.types_dict:
            type_list.append(self.types_dict[type_name])

        return type_list

    def save_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save or update a pddl type

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        self.types_dict[pddl_type_dto.get_type_name()] = pddl_type_dto

        return True

    def delete_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ delete a pddl type
            if not exists, returns false

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        if not pddl_type_dto.get_type_name() in self.types_dict:
            return False

        del self.types_dict[pddl_type_dto.get_type_name()]

        return True

    def delete_all_types(self) -> bool:
        """ delete a all pddl types

        Returns:
            bool: succeed
        """

        self.types_dict = {}

        return True

    def get_object(self, object_name: str) -> PddlObjectDto:
        """ get the PddlObjectDto with a given object name

        Args:
            object_name (str): object name

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        if object_name in self.objects_dict:
            return self.objects_dict[object_name]

        return None

    def get_all_objects(self) -> List[PddlObjectDto]:
        """ get all PddlObjectDto

        Returns:
            List[PddlObjectDto]: list of PddlObjectDto
        """

        object_list = []

        for object_name in self.objects_dict:
            object_list.append(self.objects_dict[object_name])

        return object_list

    def save_object(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save or update a pddl object

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: succeed
        """

        self.objects_dict[pddl_object_dto.get_object_name()] = pddl_object_dto

        return True

    def delete_object(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ delete a pddl object
            if not exists, returns false

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: succeed
        """

        if not pddl_object_dto.get_object_name() in self.objects_dict:
            return False

        del self.objects_dict[pddl_object_dto.get_object_name()]

        return True

    def delete_all_objects(self) -> bool:
        """ delete a all pddl objects

        Returns:
            bool: succeed
        """

        self.objects_dict = {}

        return True

    def get_predicate(self, predicate_name: str) -> PddlPredicateDto:
        """ get the PddlPredicateDto with a given predicate name

        Args:
            predicate_name (str): predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        if predicate_name in self.predicates_dict:
            return self.predicates_dict[predicate_name]

        return None

    def get_all_predicates(self) -> List[PddlPredicateDto]:
        """ get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of PddlPredicateDto
        """

        predicate_list = []

        for predicate_name in self.predicates_dict:
            predicate_list.append(self.predicates_dict[predicate_name])

        return predicate_list

    def save_predicate(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save or update a pddl predicate

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: succeed
        """

        self.predicates_dict[pddl_predicate_dto.get_predicate_name(
        )] = pddl_predicate_dto

        return True

    def delete_predicate(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ delete a pddl predicate
            if not exists, returns false

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: succeed
        """

        if not pddl_predicate_dto.get_predicate_name() in self.predicates_dict:
            return False

        del self.predicates_dict[pddl_predicate_dto.get_predicate_name()]

        return True

    def delete_all_predicates(self) -> bool:
        """ delete a all pddl predicates

        Returns:
            bool: succeed
        """

        self.predicates_dict = {}

        return True

    def get_action(self, action_name: str) -> PddlActionDto:
        """ get the PddlActionDto with a given action name

        Args:
            action_name (str): action name

        Returns:
            PddlActionDto: PddlActionDto
        """

        if action_name in self.actions_dict:
            return self.actions_dict[action_name]

        return None

    def get_all_actions(self) -> List[PddlActionDto]:
        """ get all PddlActionDto

        Returns:
            List[PddlActionDto]: list of PddlActionDto
        """

        action_list = []

        for action_name in self.actions_dict:
            action_list.append(self.actions_dict[action_name])

        return action_list

    def save_action(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save or update a pddl action

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            bool: succeed
        """

        self.actions_dict[pddl_action_dto.get_action_name(
        )] = pddl_action_dto

        return True

    def delete_action(self, pddl_action_dto: PddlActionDto) -> bool:
        """ delete a pddl action
            if not exists, returns false

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            bool: succeed
        """

        if not pddl_action_dto.get_action_name() in self.actions_dict:
            return False

        del self.actions_dict[pddl_action_dto.get_action_name()]

        return True

    def delete_all_actions(self) -> bool:
        """ delete a all pddl actions

        Returns:
            bool: succeed
        """

        self.actions_dict = {}

        return True

    def get_propositions(self, predicate_name: str) -> List[PddlPropositionDto]:
        """ get the PddlPropositionDto with a given predicate name

        Args:
            action_name (str): action name

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        proposition_list = []

        for proposition in self.propositions_list:
            if proposition.get_pddl_predicate().get_predicate_name() == predicate_name:
                proposition_list.append(proposition)

        return proposition_list

    def get_propositions_goals(self) -> List[PddlPropositionDto]:
        """ get all goal PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        proposition_list = []

        for proposition in self.propositions_list:
            if proposition.get_is_goal():
                proposition_list.append(proposition)

        return proposition_list

    def get_propositions_no_goals(self) -> List[PddlPropositionDto]:
        """ get all no goal PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        proposition_list = []

        for proposition in self.propositions_list:
            if not proposition.get_is_goal():
                proposition_list.append(proposition)

        return proposition_list

    def get_all_propositions(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self.propositions_list

    def save_proposition(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save or update a pddl action

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: succeed
        """

        self.propositions_list.append(pddl_proposition_dto)

        return True

    def delete_proposition(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ delete a pddl proposition
            if not exists, returns false

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: succeed
        """

        if not pddl_proposition_dto in self.propositions_list:
            return False

        self.propositions_list.remove(pddl_proposition_dto)

        return True

    def delete_all_propositions(self) -> bool:
        """ delete a all pddl propositions

        Returns:
            bool: succeed
        """

        self.propositions_list = []

        return True
