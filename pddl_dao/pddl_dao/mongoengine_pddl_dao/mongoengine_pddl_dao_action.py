
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_action import PddlDaoAction
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlActionModel as mongoengine_pddl_action_model
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlConditionEffectModel as mongoengine_pddl_condition_effect_model
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlParameterModel as mongoengine_pddl_parameter_model

from pddl_dao.pddl_dto.pddl_dto_action import PddlDtoAction
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject
from pddl_dao.pddl_dto.pddl_dto_condition_efect import PddlDtoConditionEffect
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import MongoenginePddlDaoType
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_proposition import MongoenginePddlDaoProposition


class Mongoengine_PDDL_DAO_Action(PddlDaoAction, MongoenginePddlDao):

    def __init__(self, uri: str = None):

        PddlDaoAction.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._mongoengine_pddl_dao_type = MongoenginePddlDaoType(uri)
        self._mongoengine_pddl_dao_proposition = MongoenginePddlDaoProposition(
            uri)
        self._mongoengine_pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(
            uri)

    def __condition_effect_mongoengine_to_dto(self, mongoengine_pddl_condition_effect: mongoengine_pddl_condition_effect_model, parameter_dict: dict) -> PddlDtoConditionEffect:

        pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(self.get_uri())

        pddl_dto_precicate = pddl_dao_predicate.get(
            mongoengine_pddl_condition_effect.PddlPredicateModel.predicate_name)

        pddl_dto_condition_effect = PddlDtoConditionEffect(
            mongoengine_pddl_condition_effect.time,
            pddl_dto_precicate,
            is_negative=mongoengine_pddl_condition_effect.is_negative)

        pddl_dto_objects_list = []

        for param in mongoengine_pddl_condition_effect.pddl_parameters:
            pddl_dto_object = parameter_dict[param.parameter_name]
            pddl_dto_objects_list.append(pddl_dto_object)

        pddl_dto_condition_effect.set_pddl_objects_list(pddl_dto_objects_list)

        return pddl_dto_condition_effect

    def _mongoengine_to_dto(self, mongoengine_pddl_action: mongoengine_pddl_action_model) -> PddlDtoAction:

        pddl_dto_action = PddlDtoAction(
            mongoengine_pddl_action.action_name)
        pddl_dto_action.set_duration(mongoengine_pddl_action.duration)
        pddl_dto_action.set_durative(mongoengine_pddl_action.durative)

        parameters_list = []
        conditions_list = []
        effects_list = []
        parameter_dict = {}

        # ACTION PARAMS
        for param in mongoengine_pddl_action.pddl_parameters:
            pddl_dto_type = PddlDtoType(param.PddlTypeModel.type_name)
            pddl_dto_object = PddlDtoObject(
                pddl_dto_type, param.parameter_name)
            parameter_dict[param.parameter_name] = pddl_dto_object
            parameters_list.append(pddl_dto_object)

        # ACTION CONDIS
        for condi in mongoengine_pddl_action.conditions:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                condi, parameter_dict)
            conditions_list.append(pddl_dto_condition_effect)

        # ACTION EFFECTS
        for effect in mongoengine_pddl_action.effects:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                effect, parameter_dict)
            effects_list.append(pddl_dto_condition_effect)

        pddl_dto_action.set_parameters_list(parameters_list)
        pddl_dto_action.set_conditions_list(conditions_list)
        pddl_dto_action.set_effects_list(effects_list)

        return pddl_dto_action

    def __condition_effect_dto_to_mongoengine(self, pddl_dto_condition_effect: PddlDtoConditionEffect, parameter_dict: dict) -> mongoengine_pddl_condition_effect_model:

        mongoengine_pddl_predicate = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_condition_effect.get_pddl_predicate())

        # check if predicate exists
        if(not mongoengine_pddl_predicate):
            return None

        mongoengine_pddl_condi = mongoengine_pddl_condition_effect_model()
        mongoengine_pddl_condi.PddlPredicateModel = mongoengine_pddl_predicate
        mongoengine_pddl_condi.time = pddl_dto_condition_effect.get_time()
        mongoengine_pddl_condi.is_negative = pddl_dto_condition_effect.get_is_negative()

        for param in pddl_dto_condition_effect.get_pddl_objects_list():

            mongoengine_pddl_condi.pddl_parameters.append(
                parameter_dict[param.get_object_name()])

        return mongoengine_pddl_condi

    def _dto_to_mongoengine(self, pddl_dto_action: PddlDtoAction) -> mongoengine_pddl_action_model:

        mongoengine_pddl_action = mongoengine_pddl_action_model()

        mongoengine_pddl_action.action_name = pddl_dto_action.get_action_name()
        mongoengine_pddl_action.duration = pddl_dto_action.get_duration()
        mongoengine_pddl_action.durative = pddl_dto_action.get_durative()

        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_dto_action.get_parameters_list():
            mongoengine_pddl_type = self._mongoengine_pddl_dao_type._get_mongoengine(
                param.get_pddl_type())

            # check if type exists
            if(not mongoengine_pddl_type):
                return None

            param_name = param.get_object_name()

            mongoengine_pddl_parameter = mongoengine_pddl_parameter_model()
            mongoengine_pddl_parameter.parameter_name = param_name
            mongoengine_pddl_parameter.PddlTypeModel = mongoengine_pddl_type

            mongoengine_pddl_action.pddl_parameters.append(
                mongoengine_pddl_parameter)

            parameter_dict[param_name] = mongoengine_pddl_parameter

        # ACTION CONDIS
        for pddl_dto_condition in pddl_dto_action.get_conditions_list():
            mongoengine_pddl_condi = self.__condition_effect_dto_to_mongoengine(
                pddl_dto_condition, parameter_dict)

            if(not mongoengine_pddl_condi):
                return None

            mongoengine_pddl_action.conditions.append(mongoengine_pddl_condi)
            mongoengine_pddl_action._pddl_predicates_used.append(
                mongoengine_pddl_condi.PddlPredicateModel)

        # ACTION EFFECTS
        for pddl_dto_effect in pddl_dto_action.get_effects_list():
            mongoengine_pddl_effect = self.__condition_effect_dto_to_mongoengine(
                pddl_dto_effect, parameter_dict)

            if(not mongoengine_pddl_effect):
                return None

            mongoengine_pddl_action.effects.append(mongoengine_pddl_effect)
            mongoengine_pddl_action._pddl_predicates_used.append(
                mongoengine_pddl_effect.PddlPredicateModel)

        return mongoengine_pddl_action

    def _exist_in_mongo(self, pddl_dto_action):

        if(self._get_mongoengine(pddl_dto_action)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_action: PddlDtoAction) -> mongoengine_pddl_action_model:

        mongoengine_pddl_action = mongoengine_pddl_action_model.objects(
            action_name=pddl_dto_action.get_action_name())
        if(not mongoengine_pddl_action):
            return None
        return mongoengine_pddl_action[0]

    def _check_pddl_dto_action_is_correct(self, pddl_dto_action: PddlDtoAction) -> bool:

        for condition in pddl_dto_action.get_conditions_list():
            if(not self._mongoengine_pddl_dao_proposition._check_pddl_dto_proposition_is_correct(condition)):
                return False

        for effect in pddl_dto_action.get_effects_list():
            if(not self._mongoengine_pddl_dao_proposition._check_pddl_dto_proposition_is_correct(effect)):
                return False

        return True

    def _check_mongoengine_pddl_condition_efect_is_correct(self, mongoengine_pddl_condition_effect: mongoengine_pddl_condition_effect_model) -> bool:

        # check if proposition is correct
        if(len(mongoengine_pddl_condition_effect.pddl_parameters) !=
           len(mongoengine_pddl_condition_effect.PddlPredicateModel.pddl_types)):
            return False

        for pddl_param, PddlTypeModel in zip(mongoengine_pddl_condition_effect.pddl_parameters,
                                             mongoengine_pddl_condition_effect.PddlPredicateModel.pddl_types):

            # check if proposition is correct
            if(pddl_param.PddlTypeModel.type_name != PddlTypeModel.type_name):
                return False

        return True

    def _check_mongoengine_pddl_action_is_correct(self, mongoengine_pddl_action: mongoengine_pddl_action_model) -> bool:

        for condition in mongoengine_pddl_action.conditions:
            if(not self._check_mongoengine_pddl_condition_efect_is_correct(condition)):
                return False

        for effect in mongoengine_pddl_action.effects:
            if(not self._check_mongoengine_pddl_condition_efect_is_correct(effect)):
                return False

        return True

    def get(self, action_name: str) -> PddlDtoAction:

        mongoengine_pddl_action = mongoengine_pddl_action_model.objects(
            action_name=action_name)

        # check if action exists
        if(mongoengine_pddl_action):

            mongoengine_pddl_action = mongoengine_pddl_action[0]

            if(not self._check_mongoengine_pddl_action_is_correct(mongoengine_pddl_action)):
                return None

            pddl_dto_action = self._mongoengine_to_dto(
                mongoengine_pddl_action)
            return pddl_dto_action

        else:
            return None

    def get_all(self) -> List[PddlDtoAction]:

        mongoengine_pddl_action = mongoengine_pddl_action_model.objects
        pddl_dto_action_list = []

        for ele in mongoengine_pddl_action:
            if(self._check_mongoengine_pddl_action_is_correct(ele)):
                pddl_dto_action = self._mongoengine_to_dto(ele)
                pddl_dto_action_list.append(pddl_dto_action)

        return pddl_dto_action_list

    def _save(self, pddl_dto_action: PddlDtoAction) -> bool:

        if(not self._check_pddl_dto_action_is_correct(pddl_dto_action)):
            return False

       # propagating saving
        for pddl_dto_type in pddl_dto_action.get_parameters_list():
            result = self._mongoengine_pddl_dao_type.save(
                pddl_dto_type.get_pddl_type())
            if(not result):
                return False
        for pddl_dto_condition in pddl_dto_action.get_conditions_list():
            result = self._mongoengine_pddl_dao_predicate.save(
                pddl_dto_condition.get_pddl_predicate())
            if(not result):
                return False
        for pddl_dto_effect in pddl_dto_action.get_effects_list():
            result = self._mongoengine_pddl_dao_predicate.save(
                pddl_dto_effect.get_pddl_predicate())
            if(not result):
                return False

        if(self._exist_in_mongo(pddl_dto_action)):
            return False

        mongoengine_pddl_action = self._dto_to_mongoengine(
            pddl_dto_action)

        if(mongoengine_pddl_action):
            mongoengine_pddl_action.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_action: PddlDtoAction) -> bool:

        if(not self._check_pddl_dto_action_is_correct(pddl_dto_action)):
            return False

        mongoengine_pddl_action = self._get_mongoengine(pddl_dto_action)

        # check if action exists
        if(mongoengine_pddl_action):
            new_pddl_action_mongoengine = self._dto_to_mongoengine(
                pddl_dto_action)

            if(new_pddl_action_mongoengine):
                mongoengine_pddl_action.action_name = new_pddl_action_mongoengine.action_name
                mongoengine_pddl_action.durative = new_pddl_action_mongoengine.durative
                mongoengine_pddl_action.duration = new_pddl_action_mongoengine.duration
                mongoengine_pddl_action.pddl_parameters = new_pddl_action_mongoengine.pddl_parameters
                mongoengine_pddl_action.conditions = new_pddl_action_mongoengine.conditions
                mongoengine_pddl_action.effects = new_pddl_action_mongoengine.effects
                mongoengine_pddl_action.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_action: PddlDtoAction) -> bool:

        if(self._exist_in_mongo(pddl_dto_action)):
            return self._update(pddl_dto_action)

        else:
            return self._save(pddl_dto_action)

    def delete(self, pddl_dto_action):

        mongoengine_pddl_action = self._get_mongoengine(pddl_dto_action)

        # check if action exists
        if(mongoengine_pddl_action):
            mongoengine_pddl_action.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_action_list = self.get_all()

        for ele in pddl_dto_action_list:
            self.delete(ele)

        return True
