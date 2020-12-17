
""" Dto Msg Parser """

from merlin2_knowledge_base_interfaces.msg import PddlType
from merlin2_knowledge_base_interfaces.msg import PddlObject
from merlin2_knowledge_base_interfaces.msg import PddlPredicate
from merlin2_knowledge_base_interfaces.msg import PddlProposition
from merlin2_knowledge_base_interfaces.msg import PddlConditionEffect
from merlin2_knowledge_base_interfaces.msg import PddlAction

from pddl_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlActionDto
)


class DtoMsgParser:
    """ Dto Msg Parser Class """

    def type_dto_to_msg(self, pddl_type_dto: PddlTypeDto) -> PddlType:
        """ convert a PddlTypeDto into a PddlType msg

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            PddlType: PddlType msg
        """

        msg = PddlType()

        msg.type_name = pddl_type_dto.get_type_name()

        return msg

    def object_dto_to_msg(self, pddl_object_dto: PddlObjectDto) -> PddlObject:
        """ convert a PddlObjectDto into a PddlObject msg

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            PddlObject: PddlObject msg
        """

        msg = PddlObject()

        msg.pddl_type = self.type_dto_to_msg(pddl_object_dto.get_pddl_type())
        msg.object_name = pddl_object_dto.get_object_name()

        return msg

    def predicate_dto_to_msg(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicate:
        """ convert a PddlPredicateDto into a PddlPredicate msg

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            PddlPredicate: PddlPredicate msg
        """

        msg = PddlPredicate()

        msg.predicate_name = pddl_predicate_dto.get_predicate_name()

        msg.pddl_types = []
        for pddl_type_dto in pddl_predicate_dto.get_pddl_types_list():
            msg.pddl_types.append(self.type_dto_to_msg(pddl_type_dto))

        return msg

    def proposition_dto_to_msg(self, pddl_proposition_dto: PddlPropositionDto) -> PddlProposition:
        """ convert a PddlPropositionDto into a PddlProposition msg

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            PddlProposition: PddlProposition msg
        """

        msg = PddlProposition()

        msg.pddl_predicate = self.predicate_dto_to_msg(
            pddl_proposition_dto.get_pddl_predicate())

        msg.is_goal = pddl_proposition_dto.get_is_goal()

        msg.pddl_objects = []
        for pddl_object_dto in pddl_proposition_dto.get_pddl_objects_list():
            msg.pddl_objects.append(self.object_dto_to_msg(pddl_object_dto))

        return msg

    def condition_effect_dto_to_msg(self,
                                    pddl_condition_effect_dto:
                                        PddlConditionEffectDto) -> PddlConditionEffect:
        """ convert a PddlConditionEffectDto into a PddlConditionEffect msg

        Args:
            pddl_condition_effect_dto (PddlConditionEffectDto): PddlConditionEffectDto

        Returns:
            PddlConditionEffect: PddlConditionEffect msg
        """

        msg = PddlConditionEffect()

        msg.pddl_predicate = self.predicate_dto_to_msg(
            pddl_condition_effect_dto.get_pddl_predicate())

        msg.pddl_objects = []
        for pddl_object_dto in pddl_condition_effect_dto.get_pddl_objects_list():
            msg.pddl_objects.append(self.object_dto_to_msg(pddl_object_dto))

        if pddl_condition_effect_dto.get_time():
            msg.time = pddl_condition_effect_dto.get_time()
        msg.is_negative = pddl_condition_effect_dto.get_is_negative()

        return msg

    def action_dto_to_msg(self, pddl_action_dto: PddlActionDto) -> PddlAction:
        """ convert a PddlActionDto into a PddlAction msg

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            PddlAction: PddlAction msg
        """

        msg = PddlAction()

        msg.action_name = pddl_action_dto.get_action_name()
        msg.duration = pddl_action_dto.get_duration()
        msg.durative = pddl_action_dto.get_durative()

        msg.pddl_parameters = []
        for pddl_object_dto in pddl_action_dto.get_pddl_parameters_list():
            msg.pddl_parameters.append(self.object_dto_to_msg(pddl_object_dto))

        msg.pddl_coditions = []
        for pddl_condition_dto in pddl_action_dto.get_pddl_conditions_list():
            msg.pddl_coditions.append(
                self.condition_effect_dto_to_msg(pddl_condition_dto))

        msg.pddl_effects = []
        for pddl_effect_dto in pddl_action_dto.get_pddl_effects_list():
            msg.pddl_effects.append(
                self.condition_effect_dto_to_msg(pddl_effect_dto))

        return msg
