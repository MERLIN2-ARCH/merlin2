
""" Mongoengine Pddl Dao Action """


from typing import List

from pddl_dao.pddl_dao_interface.pddl_action_dao import PddlActionDao
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlActionModel
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlConditionEffectModel
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlParameterModel

from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_proposition_dao import(
    MongoenginePddlPropositionDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_type_dao import(
    MongoenginePddlTypeDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_predicate_dao import (
    MongoenginePddlPredicateDao
)


class MongoenginePddlDaoAction(PddlActionDao, MongoenginePddlDao):
    """ Mongoengine Pddl Dao Action Class """

    def __init__(self, uri: str = None):

        PddlActionDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._me_pddl_type_dao = MongoenginePddlTypeDao(uri)
        self._me_pddl_proposition_dao = MongoenginePddlPropositionDao(uri)
        self._me_pddl_predicate_dao = MongoenginePddlPredicateDao(uri)

    def __condition_effect_mongoengine_to_dto(self,
                                              pddl_condition_effect_model: PddlConditionEffectModel,
                                              parameter_dict: dict) -> PddlConditionEffectDto:

        pddl_predicate_dao = MongoenginePddlPredicateDao(self.get_uri())

        pddl_dto_precicate = pddl_predicate_dao.get(
            pddl_condition_effect_model.PddlPredicateModel.predicate_name)

        pddl_dto_condition_effect = PddlConditionEffectDto(
            pddl_condition_effect_model.time,
            pddl_dto_precicate,
            is_negative=pddl_condition_effect_model.is_negative)

        pddl_dto_objects_list = []

        for param in pddl_condition_effect_model.pddl_parameters:
            pddl_object_dto = parameter_dict[param.parameter_name]
            pddl_dto_objects_list.append(pddl_object_dto)

        pddl_dto_condition_effect.set_pddl_objects_list(pddl_dto_objects_list)

        return pddl_dto_condition_effect

    def _model_to_dto(self, pddl_action_model: PddlActionModel) -> PddlActionDto:

        pddl_action_dto = PddlActionDto(
            pddl_action_model.action_name)
        pddl_action_dto.set_duration(pddl_action_model.duration)
        pddl_action_dto.set_durative(pddl_action_model.durative)

        parameters_list = []
        conditions_list = []
        effects_list = []
        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_action_model.pddl_parameters:
            pddl_type_dto = PddlTypeDto(param.PddlTypeModel.type_name)
            pddl_object_dto = PddlObjectDto(
                pddl_type_dto, param.parameter_name)
            parameter_dict[param.parameter_name] = pddl_object_dto
            parameters_list.append(pddl_object_dto)

        # ACTION CONDIS
        for condi in pddl_action_model.conditions:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                condi, parameter_dict)
            conditions_list.append(pddl_dto_condition_effect)

        # ACTION EFFECTS
        for effect in pddl_action_model.effects:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                effect, parameter_dict)
            effects_list.append(pddl_dto_condition_effect)

        pddl_action_dto.set_parameters_list(parameters_list)
        pddl_action_dto.set_conditions_list(conditions_list)
        pddl_action_dto.set_effects_list(effects_list)

        return pddl_action_dto

    def __condition_effect_dto_to_mongoengine(self,
                                              pddl_dto_condition_effect: PddlConditionEffectDto,
                                              parameter_dict: dict) -> PddlConditionEffectModel:

        pddl_predicate_model = self._me_pddl_predicate_dao._get_model(
            pddl_dto_condition_effect.get_pddl_predicate())

        # check if predicate exists
        if not pddl_predicate_model:
            return None

        mongoengine_pddl_condi = PddlConditionEffectModel()
        mongoengine_pddl_condi.PddlPredicateModel = pddl_predicate_model
        mongoengine_pddl_condi.time = pddl_dto_condition_effect.get_time()
        mongoengine_pddl_condi.is_negative = pddl_dto_condition_effect.get_is_negative()

        for param in pddl_dto_condition_effect.get_pddl_objects_list():

            mongoengine_pddl_condi.pddl_parameters.append(
                parameter_dict[param.get_object_name()])

        return mongoengine_pddl_condi

    def _dto_to_model(self, pddl_action_dto: PddlActionDto) -> PddlActionModel:

        pddl_action_model = PddlActionModel()

        pddl_action_model.action_name = pddl_action_dto.get_action_name()
        pddl_action_model.duration = pddl_action_dto.get_duration()
        pddl_action_model.durative = pddl_action_dto.get_durative()

        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_action_dto.get_parameters_list():
            pddl_type_model = self._me_pddl_type_dao._get_model(
                param.get_pddl_type())

            # check if type exists
            if not pddl_type_model:
                return None

            param_name = param.get_object_name()

            mongoengine_pddl_parameter = PddlParameterModel()
            mongoengine_pddl_parameter.parameter_name = param_name
            mongoengine_pddl_parameter.PddlTypeModel = pddl_type_model

            pddl_action_model.pddl_parameters.append(
                mongoengine_pddl_parameter)

            parameter_dict[param_name] = mongoengine_pddl_parameter

        # ACTION CONDIS
        for pddl_dto_condition in pddl_action_dto.get_conditions_list():
            mongoengine_pddl_condi = self.__condition_effect_dto_to_mongoengine(
                pddl_dto_condition, parameter_dict)

            if not mongoengine_pddl_condi:
                return None

            pddl_action_model.conditions.append(mongoengine_pddl_condi)
            pddl_action_model._pddl_predicates_used.append(
                mongoengine_pddl_condi.PddlPredicateModel)

        # ACTION EFFECTS
        for pddl_dto_effect in pddl_action_dto.get_effects_list():
            mongoengine_pddl_effect = self.__condition_effect_dto_to_mongoengine(
                pddl_dto_effect, parameter_dict)

            if not mongoengine_pddl_effect:
                return None

            pddl_action_model.effects.append(mongoengine_pddl_effect)
            pddl_action_model._pddl_predicates_used.append(
                mongoengine_pddl_effect.PddlPredicateModel)

        return pddl_action_model

    def _exist_in_mongo(self, pddl_action_dto):

        if self._get_model(pddl_action_dto):
            return True
        return False

    def _get_model(self, pddl_action_dto: PddlActionDto) -> PddlActionModel:

        pddl_action_model = PddlActionModel.objects(
            action_name=pddl_action_dto.get_action_name())
        if not pddl_action_model:
            return None
        return pddl_action_model[0]

    def _check_pddl_action_dto(self, pddl_action_dto: PddlActionDto) -> bool:

        for condition in pddl_action_dto.get_conditions_list():
            if not self._me_pddl_proposition_dao._check_pddl_proposition_dto(condition):
                return False

        for effect in pddl_action_dto.get_effects_list():
            if not self._me_pddl_proposition_dao._check_pddl_proposition_dto(effect):
                return False

        return True

    def _check_pddl_condition_efect_model(self,
                                          pddl_condition_effect_model: PddlConditionEffectModel) -> bool:

        # check if proposition is correct
        if(len(pddl_condition_effect_model.pddl_parameters) !=
           len(pddl_condition_effect_model.PddlPredicateModel.pddl_types)):
            return False

        pddl_parameter_models = pddl_condition_effect_model.pddl_parameters
        pddl_type_models = pddl_condition_effect_model.PddlPredicateModel.pddl_types

        for pddl_param, pddl_type_model in zip(pddl_parameter_models, pddl_type_models):

            # check if proposition is correct
            if pddl_param.PddlTypeModel.type_name != pddl_type_model.type_name:
                return False

        return True

    def _check_pddl_action_model(self, pddl_action_model: PddlActionModel) -> bool:

        for condition in pddl_action_model.conditions:
            if not self._check_pddl_condition_efect_model(condition):
                return False

        for effect in pddl_action_model.effects:
            if not self._check_pddl_condition_efect_model(effect):
                return False

        return True

    def get(self, action_name: str) -> PddlActionDto:

        pddl_action_model = PddlActionModel.objects(
            action_name=action_name)

        # check if action exists
        if pddl_action_model:

            pddl_action_model = pddl_action_model[0]

            if not self._check_pddl_action_model(pddl_action_model):
                return None

            pddl_action_dto = self._model_to_dto(
                pddl_action_model)
            return pddl_action_dto

        else:
            return None

    def get_all(self) -> List[PddlActionDto]:

        pddl_action_model = PddlActionModel.objects
        pddl_dto_action_list = []

        for ele in pddl_action_model:
            if self._check_pddl_action_model(ele):
                pddl_action_dto = self._model_to_dto(ele)
                pddl_dto_action_list.append(pddl_action_dto)

        return pddl_dto_action_list

    def _save(self, pddl_action_dto: PddlActionDto) -> bool:

        if not self._check_pddl_action_dto(pddl_action_dto):
            return False

       # propagating saving
        for pddl_type_dto in pddl_action_dto.get_parameters_list():
            result = self._me_pddl_type_dao.save(
                pddl_type_dto.get_pddl_type())
            if not result:
                return False
        for pddl_dto_condition in pddl_action_dto.get_conditions_list():
            result = self._me_pddl_predicate_dao.save(
                pddl_dto_condition.get_pddl_predicate())
            if not result:
                return False
        for pddl_dto_effect in pddl_action_dto.get_effects_list():
            result = self._me_pddl_predicate_dao.save(
                pddl_dto_effect.get_pddl_predicate())
            if not result:
                return False

        if self._exist_in_mongo(pddl_action_dto):
            return False

        pddl_action_model = self._dto_to_model(
            pddl_action_dto)

        if pddl_action_model:
            pddl_action_model.save()
            return True

        else:
            return False

    def _update(self, pddl_action_dto: PddlActionDto) -> bool:

        if not self._check_pddl_action_dto(pddl_action_dto):
            return False

        pddl_action_model = self._get_model(pddl_action_dto)

        # check if action exists
        if pddl_action_model:
            new_pddl_action_model = self._dto_to_model(
                pddl_action_dto)

            if new_pddl_action_model:
                pddl_action_model.action_name = new_pddl_action_model.action_name
                pddl_action_model.durative = new_pddl_action_model.durative
                pddl_action_model.duration = new_pddl_action_model.duration
                pddl_action_model.pddl_parameters = new_pddl_action_model.pddl_parameters
                pddl_action_model.conditions = new_pddl_action_model.conditions
                pddl_action_model.effects = new_pddl_action_model.effects
                pddl_action_model.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_action_dto: PddlActionDto) -> bool:

        if self._exist_in_mongo(pddl_action_dto):
            return self._update(pddl_action_dto)

        else:
            return self._save(pddl_action_dto)

    def delete(self, pddl_action_dto):

        pddl_action_model = self._get_model(pddl_action_dto)

        # check if action exists
        if pddl_action_model:
            pddl_action_model.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_action_list = self.get_all()

        for ele in pddl_dto_action_list:
            self.delete(ele)

        return True
