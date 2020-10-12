
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_action import PDDL_DAO_Action
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_action as pddl_mongoengine_action_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_condition_effect as pddl_mongoengine_condition_effect_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_parameter as pddl_mongoengine_parameter_model

from pddl_dao.pddl_dto.pddl_dto_action import PDDL_DTO_Action
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object
from pddl_dao.pddl_dto.pddl_dto_condition_efect import PDDL_DTO_ConditionEffect
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import Mongoengine_PDDL_DAO_Type
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_proposition import Mongoengine_PDDL_DAO_Proposition


class Mongoengine_PDDL_DAO_Action(PDDL_DAO_Action, Mongoengine_PDDL_DAO):

    def __init__(self, uri: str = None):

        PDDL_DAO_Action.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

        self._mongoengine_pddl_dao_type = Mongoengine_PDDL_DAO_Type(uri)
        self._mongoengine_pddl_dao_proposition = Mongoengine_PDDL_DAO_Proposition(
            uri)
        self._mongoengine_pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(
            uri)

    def __condition_effect_mongoengine_to_dto(self, pddl_mongoengine_condition_effect: pddl_mongoengine_condition_effect_model, parameter_dict: dict) -> PDDL_DTO_ConditionEffect:

        pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(self.get_uri())

        pddl_dto_precicate = pddl_dao_predicate.get(
            pddl_mongoengine_condition_effect.pddl_predicate.predicate_name)

        pddl_dto_condition_effect = PDDL_DTO_ConditionEffect(
            pddl_mongoengine_condition_effect.time,
            pddl_dto_precicate,
            is_negative=pddl_mongoengine_condition_effect.is_negative)

        pddl_dto_objects_list = []

        for param in pddl_mongoengine_condition_effect.pddl_parameters:
            pddl_dto_object = parameter_dict[param.parameter_name]
            pddl_dto_objects_list.append(pddl_dto_object)

        pddl_dto_condition_effect.set_pddl_objects_list(pddl_dto_objects_list)

        return pddl_dto_condition_effect

    def _mongoengine_to_dto(self, pddl_mongoengine_action: pddl_mongoengine_action_model) -> PDDL_DTO_Action:

        pddl_dto_action = PDDL_DTO_Action(
            pddl_mongoengine_action.action_name)
        pddl_dto_action.set_duration(pddl_mongoengine_action.duration)
        pddl_dto_action.set_durative(pddl_mongoengine_action.durative)

        parameters_list = []
        conditions_list = []
        effects_list = []
        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_mongoengine_action.pddl_parameters:
            pddl_dto_type = PDDL_DTO_Type(param.pddl_type.type_name)
            pddl_dto_object = PDDL_DTO_Object(
                pddl_dto_type, param.parameter_name)
            parameter_dict[param.parameter_name] = pddl_dto_object
            parameters_list.append(pddl_dto_object)

        # ACTION CONDIS
        for condi in pddl_mongoengine_action.conditions:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                condi, parameter_dict)
            conditions_list.append(pddl_dto_condition_effect)

        # ACTION EFFECTS
        for effect in pddl_mongoengine_action.effects:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                effect, parameter_dict)
            effects_list.append(pddl_dto_condition_effect)

        pddl_dto_action.set_parameters_list(parameters_list)
        pddl_dto_action.set_conditions_list(conditions_list)
        pddl_dto_action.set_effects_list(effects_list)

        return pddl_dto_action

    def __condition_effect_dto_to_mongoengine(self, pddl_dto_condition_effect: PDDL_DTO_ConditionEffect, parameter_dict: dict) -> pddl_mongoengine_condition_effect_model:

        pddl_mongoengine_predicate = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_condition_effect.get_pddl_predicate())

        # check if predicate exists
        if(not pddl_mongoengine_predicate):
            return None

        pddl_mongoengine_condi = pddl_mongoengine_condition_effect_model()
        pddl_mongoengine_condi.pddl_predicate = pddl_mongoengine_predicate
        pddl_mongoengine_condi.time = pddl_dto_condition_effect.get_time()
        pddl_mongoengine_condi.is_negative = pddl_dto_condition_effect.get_is_negative()

        for param in pddl_dto_condition_effect.get_pddl_objects_list():

            pddl_mongoengine_condi.pddl_parameters.append(
                parameter_dict[param.get_object_name()])

        return pddl_mongoengine_condi

    def _dto_to_mongoengine(self, pddl_dto_action: PDDL_DTO_Action) -> pddl_mongoengine_action_model:

        pddl_mongoengine_action = pddl_mongoengine_action_model()

        pddl_mongoengine_action.action_name = pddl_dto_action.get_action_name()
        pddl_mongoengine_action.duration = pddl_dto_action.get_duration()
        pddl_mongoengine_action.durative = pddl_dto_action.get_durative()

        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_dto_action.get_parameters_list():
            pddl_mongoengine_type = self._mongoengine_pddl_dao_type._get_mongoengine(
                param.get_pddl_type())

            # check if type exists
            if(not pddl_mongoengine_type):
                return None

            param_name = param.get_object_name()

            pddl_mongoengine_parameter = pddl_mongoengine_parameter_model()
            pddl_mongoengine_parameter.parameter_name = param_name
            pddl_mongoengine_parameter.pddl_type = pddl_mongoengine_type

            pddl_mongoengine_action.pddl_parameters.append(
                pddl_mongoengine_parameter)

            parameter_dict[param_name] = pddl_mongoengine_parameter

        # ACTION CONDIS
        for condi in pddl_dto_action.get_conditions_list():
            pddl_mongoengine_condi = self.__condition_effect_dto_to_mongoengine(
                condi, parameter_dict)

            if(not pddl_mongoengine_condi):
                return None

            pddl_mongoengine_action.conditions.append(pddl_mongoengine_condi)
            pddl_mongoengine_action._pddl_predicates_used.append(
                pddl_mongoengine_condi.pddl_predicate)

        # ACTION EFFECTS
        for effect in pddl_dto_action.get_effects_list():
            pddl_mongoengine_effect = self.__condition_effect_dto_to_mongoengine(
                effect, parameter_dict)

            if(not pddl_mongoengine_effect):
                return None

            pddl_mongoengine_action.effects.append(pddl_mongoengine_effect)
            pddl_mongoengine_action._pddl_predicates_used.append(
                pddl_mongoengine_effect.pddl_predicate)

        return pddl_mongoengine_action

    def _exist_in_mongo(self, pddl_dto_action):

        if(self._get_mongoengine(pddl_dto_action)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_action: PDDL_DTO_Action) -> pddl_mongoengine_action_model:

        pddl_mongoengine_action = pddl_mongoengine_action_model.objects(
            action_name=pddl_dto_action.get_action_name())
        if(not pddl_mongoengine_action):
            return None
        return pddl_mongoengine_action[0]

    def _check_pddl_dto_action_is_correct(self, pddl_dto_action: PDDL_DTO_Action) -> bool:

        for condition in pddl_dto_action.get_conditions_list():
            if(not self._mongoengine_pddl_dao_proposition._check_pddl_dto_proposition_is_correct(condition)):
                return False

        for effect in pddl_dto_action.get_effects_list():
            if(not self._mongoengine_pddl_dao_proposition._check_pddl_dto_proposition_is_correct(effect)):
                return False

        return True

    def _check_pddl_mongoengine_condition_efect_is_correct(self, pddl_mongoengine_condition_effect: pddl_mongoengine_condition_effect_model) -> bool:

        # check if proposition is correct
        if(len(pddl_mongoengine_condition_effect.pddl_parameters) !=
           len(pddl_mongoengine_condition_effect.pddl_predicate.pddl_types)):
            return False

        for pddl_param, pddl_type in zip(pddl_mongoengine_condition_effect.pddl_parameters,
                                         pddl_mongoengine_condition_effect.pddl_predicate.pddl_types):

            # check if proposition is correct
            if(pddl_param.pddl_type.type_name != pddl_type.type_name):
                return False

        return True

    def _check_pddl_mongoengine_action_is_correct(self, pddl_mongoengine_action: pddl_mongoengine_action_model) -> bool:

        for condition in pddl_mongoengine_action.conditions:
            if(not self._check_pddl_mongoengine_condition_efect_is_correct(condition)):
                return False

        for effect in pddl_mongoengine_action.effects:
            if(not self._check_pddl_mongoengine_condition_efect_is_correct(effect)):
                return False

        return True

    def get(self, action_name: str) -> PDDL_DTO_Action:

        pddl_mongoengine_action = pddl_mongoengine_action_model.objects(
            action_name=action_name)

        # check if action exists
        if(pddl_mongoengine_action):

            pddl_mongoengine_action = pddl_mongoengine_action[0]

            if(not self._check_pddl_mongoengine_action_is_correct(pddl_mongoengine_action)):
                return None

            pddl_dto_action = self._mongoengine_to_dto(
                pddl_mongoengine_action)
            return pddl_dto_action

        else:
            return None

    def get_all(self) -> List[PDDL_DTO_Action]:

        pddl_mongoengine_action = pddl_mongoengine_action_model.objects
        pddl_dto_action_list = []

        for ele in pddl_mongoengine_action:
            if(self._check_pddl_mongoengine_action_is_correct(ele)):
                pddl_dto_action = self._mongoengine_to_dto(ele)
                pddl_dto_action_list.append(pddl_dto_action)

        return pddl_dto_action_list

    def _save(self, pddl_dto_action: PDDL_DTO_Action) -> bool:

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

        pddl_mongoengine_action = self._dto_to_mongoengine(
            pddl_dto_action)

        if(pddl_mongoengine_action):
            pddl_mongoengine_action.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_action: PDDL_DTO_Action) -> bool:

        if(not self._check_pddl_dto_action_is_correct(pddl_dto_action)):
            return False

        pddl_mongoengine_action = self._get_mongoengine(pddl_dto_action)

        # check if action exists
        if(pddl_mongoengine_action):
            new_pddl_action_mongoengine = self._dto_to_mongoengine(
                pddl_dto_action)

            if(new_pddl_action_mongoengine):
                pddl_mongoengine_action.action_name = new_pddl_action_mongoengine.action_name
                pddl_mongoengine_action.durative = new_pddl_action_mongoengine.durative
                pddl_mongoengine_action.duration = new_pddl_action_mongoengine.duration
                pddl_mongoengine_action.pddl_parameters = new_pddl_action_mongoengine.pddl_parameters
                pddl_mongoengine_action.conditions = new_pddl_action_mongoengine.conditions
                pddl_mongoengine_action.effects = new_pddl_action_mongoengine.effects
                pddl_mongoengine_action.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_action: PDDL_DTO_Action) -> bool:

        if(self._exist_in_mongo(pddl_dto_action)):
            return self._update(pddl_dto_action)

        else:
            return self._save(pddl_dto_action)

    def delete(self, pddl_dto_action):

        pddl_mongoengine_action = self._get_mongoengine(pddl_dto_action)

        # check if action exists
        if(pddl_mongoengine_action):
            pddl_mongoengine_action.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_action_list = self.get_all()

        for ele in pddl_dto_action_list:
            self.delete(ele)

        return True
