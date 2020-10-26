
""" Msg Dto Parser """

from merlin2_knowledge_base_interfaces.msg import PddlType
from merlin2_knowledge_base_interfaces.msg import PddlObject
from merlin2_knowledge_base_interfaces.msg import PddlPredicate
from merlin2_knowledge_base_interfaces.msg import PddlProposition
from merlin2_knowledge_base_interfaces.msg import PddlConditionEffect
from merlin2_knowledge_base_interfaces.msg import PddlAction

from pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dto.pddl_proposition_dto import PddlPropositionDto
from pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dto.pddl_action_dto import PddlActionDto


class MsgDtoParser:
    """ Msg Dto Parser Class """

    def type_msg_to_dto(self, pddl_type_msg: PddlType) -> PddlTypeDto:
        """ convert a PddlType msg into a PddlTypeDto

        Args:
            pddl_type_msg (PddlType): PddlType msg

        Returns:
            PddlTypeDto: PddlTypeDto
        """

        dto = PddlTypeDto(pddl_type_msg.type_name)

        return dto

    def object_msg_to_dto(self, pddl_object_msg: PddlObject) -> PddlObjectDto:
        """ convert a PddlObject msg into a PddlObjectDto

        Args:
            pddl_object_msg (PddlObject): PddlObject msg

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        dto = PddlObjectDto(
            self.type_msg_to_dto(pddl_object_msg.pddl_type),
            pddl_object_msg.object_name
        )

        return dto

    def predicate_msg_to_dto(self, pddl_predicate_msg: PddlPredicate) -> PddlPredicateDto:
        """ convert a PddlPredicate msg into a PddlPredicateDto

        Args:
            pddl_predicate_msg (PddlPredicate): PddlPredicate msg

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        dto = PddlPredicateDto(pddl_predicate_msg.predicate_name)

        pddl_types_list = []

        for pddl_type_msg in pddl_predicate_msg.pddl_types:
            pddl_types_list.append(self.type_msg_to_dto(pddl_type_msg))

        dto.set_pddl_types_list(pddl_types_list)

        return dto

    def proposition_msg_to_dto(self, pddl_proposition_msg: PddlProposition) -> PddlPropositionDto:
        """ convert a PddlProposition msg into a PddlPropositionDto

        Args:
            pddl_proposition_msg (PddlProposition): PddlProposition msg

        Returns:
            PddlPropositionDto: PddlPropositionDto
        """

        dto = PddlPropositionDto(
            self.predicate_msg_to_dto(pddl_proposition_msg.pddl_predicate)
        )

        dto.set_is_goal(pddl_proposition_msg.is_goal)

        pddl_objects_list = []

        for pddl_object_msg in pddl_proposition_msg.pddl_objects:
            pddl_objects_list.append(self.object_msg_to_dto(pddl_object_msg))

        dto.set_pddl_objects_list(pddl_objects_list)

        return dto

    def condition_effect_msg_to_dto(self,
                                    pddl_condition_efect_msg:
                                        PddlConditionEffect) -> PddlConditionEffectDto:
        """ convert a PddlConditionEffect msg into a PddlConditionEffectDto

        Args:
            pddl_condition_efect_msg (PddlConditionEffect): PddlConditionEffect msg

        Returns:
            PddlConditionEffectDto: PddlConditionEffectDto
        """

        dto = PddlConditionEffectDto(
            self.predicate_msg_to_dto(
                pddl_condition_efect_msg.pddl_predicate)
        )

        dto.set_is_negative(pddl_condition_efect_msg.is_negative)

        if pddl_condition_efect_msg.time:
            dto.set_time(pddl_condition_efect_msg.time)

        pddl_objects_list = []

        for pddl_object_msg in pddl_condition_efect_msg.pddl_objects:
            pddl_objects_list.append(self.object_msg_to_dto(pddl_object_msg))

        dto.set_pddl_objects_list(pddl_objects_list)

        return dto

    def action_msg_to_dto(self, pddl_action_msg: PddlAction) -> PddlActionDto:
        """ convert a PddlAction msg into a PddlActionDto

        Args:
            pddl_action_msg (PddlAction): PddlAction msg

        Returns:
            PddlActionDto: PddlActionDto
        """

        dto = PddlActionDto(pddl_action_msg.action_name)

        dto.set_duration(pddl_action_msg.duration)
        dto.set_durative(pddl_action_msg.durative)

        pddl_parameters_list = []
        for pddl_object_msg in pddl_action_msg.pddl_parameters:
            pddl_parameters_list.append(
                self.object_msg_to_dto(pddl_object_msg))
        dto.set_parameters_list(pddl_parameters_list)

        pddl_coditions_list = []
        for pddl_condition_msg in pddl_action_msg.pddl_coditions:
            pddl_coditions_list.append(
                self.condition_effect_msg_to_dto(pddl_condition_msg))
        dto.set_conditions_list(pddl_coditions_list)

        pddl_effect_list = []
        for pddl_effect_msg in pddl_action_msg.pddl_effects:
            pddl_effect_list.append(
                self.condition_effect_msg_to_dto(pddl_effect_msg))
        dto.set_effects_list(pddl_effect_list)

        return dto
