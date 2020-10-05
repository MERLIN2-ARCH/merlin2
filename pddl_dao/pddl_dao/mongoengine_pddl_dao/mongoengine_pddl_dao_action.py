
from pddl_dao.pddl_dao_interface.pddl_dao_action import PDDL_DAO_Action
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_action as ppddl_action_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_condition_effect as pddl_condition_effect_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_parameter as pddl_parameter_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_predicate as pddl_predicate_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_type as pddl_type_mongoengine_model

from pddl_dao.pddl_dto.pddl_dto_action import PDDL_DTO_Action
from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object
from pddl_dao.pddl_dto.pddl_dto_condition_efect import PDDL_DTO_ConditionEffect
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate


class Mongoengine_PDDL_DAO_Action(PDDL_DAO_Action, Mongoengine_PDDL_DAO):

    def __init__(self, uri=None):

        PDDL_DAO_Action.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

    def __condition_effect_mongoengine_to_dto(self, pddl_condition_effect_mongoengine, parameter_dict):

        pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(self.get_uri())

        pddl_dto_precicate = pddl_dao_predicate.get(
            pddl_condition_effect_mongoengine.pddl_predicate.predicate_name)

        pddl_dto_condition_effect = PDDL_DTO_ConditionEffect(
            pddl_condition_effect_mongoengine.time,
            pddl_dto_precicate,
            is_negative=pddl_condition_effect_mongoengine.is_negative)

        pddl_dto_objects_list = []

        for param in pddl_condition_effect_mongoengine.pddl_parameters:
            pddl_dto_object = parameter_dict[param.parameter_name]
            pddl_dto_objects_list.append(pddl_dto_object)

        pddl_dto_condition_effect.set_pddl_objects_list(pddl_dto_objects_list)

        return pddl_dto_condition_effect

    def _mongoengine_to_dto(self, pddl_action_mongoengine):

        pddl_dto_action = PDDL_DTO_Action(
            pddl_action_mongoengine.action_name)
        pddl_dto_action.set_duration(pddl_action_mongoengine.duration)
        pddl_dto_action.set_durative(pddl_action_mongoengine.durative)

        parameters_list = []
        conditions_list = []
        effects_list = []
        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_action_mongoengine.pddl_parameters:
            pddl_dto_type = PDDL_DTO_Type(param.pddl_type.type_name)
            pddl_dto_object = PDDL_DTO_Object(
                pddl_dto_type, param.parameter_name)
            parameter_dict[param.parameter_name] = pddl_dto_object
            parameters_list.append(pddl_dto_object)

        # ACTION CONDIS
        for condi in pddl_action_mongoengine.conditions:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                condi, parameter_dict)
            conditions_list.append(pddl_dto_condition_effect)

        # ACTION EFFECTS
        for effect in pddl_action_mongoengine.effects:
            pddl_dto_condition_effect = self.__condition_effect_mongoengine_to_dto(
                effect, parameter_dict)
            effects_list.append(pddl_dto_condition_effect)

        pddl_dto_action.set_parameters_list(parameters_list)
        pddl_dto_action.set_conditions_list(conditions_list)
        pddl_dto_action.set_effects_list(effects_list)

        return pddl_dto_action

    def __condition_effect_dto_to_mongoengine(self, pddl_dto_condition_effect, parameter_dict):

       # check if condition/effect is correct
        if(len(pddl_dto_condition_effect.get_pddl_objects_list()) !=
           len(pddl_dto_condition_effect.get_pddl_predicate().get_pddl_types_list())):
            return None

        pddl_predicate_mongoengine = pddl_predicate_mongoengine_model.objects(
            predicate_name=pddl_dto_condition_effect.get_pddl_predicate().get_predicate_name())

        # check if predicate exists
        if(not pddl_predicate_mongoengine):
            return None
        pddl_predicate_mongoengine = pddl_predicate_mongoengine[0]

        pddl_condi_mongoengine = pddl_condition_effect_mongoengine_model()
        pddl_condi_mongoengine.pddl_predicate = pddl_predicate_mongoengine
        pddl_condi_mongoengine.time = pddl_dto_condition_effect.get_time()
        pddl_condi_mongoengine.is_negative = pddl_dto_condition_effect.get_is_negative()

        for param, pddl_dto_type in zip(pddl_dto_condition_effect.get_pddl_objects_list(),
                                        pddl_dto_condition_effect.get_pddl_predicate().get_pddl_types_list()):

           # check if proposition is correct
            if(param.get_pddl_type().get_type_name() != pddl_dto_type.get_type_name()):
                return None

            pddl_condi_mongoengine.pddl_parameters.append(
                parameter_dict[param.get_object_name()])

        return pddl_condi_mongoengine

    def _dto_to_mongoengine(self, pddl_dto_action):

        pddl_action_mongoengine = ppddl_action_mongoengine_model()

        pddl_action_mongoengine.action_name = pddl_dto_action.get_action_name()
        pddl_action_mongoengine.duration = pddl_dto_action.get_duration()
        pddl_action_mongoengine.durative = pddl_dto_action.get_durative()

        parameter_dict = {}

        # ACTION PARAMS
        for param in pddl_dto_action.get_parameters_list():
            pddl_type_mongoengine = pddl_type_mongoengine_model.objects(
                type_name=param.get_pddl_type().get_type_name())

            # check if type exists
            if(not pddl_type_mongoengine):
                return None
            pddl_type_mongoengine = pddl_type_mongoengine[0]

            param_name = param.get_object_name()

            pddl_parameter_mongoengine = pddl_parameter_mongoengine_model()
            pddl_parameter_mongoengine.parameter_name = param_name
            pddl_parameter_mongoengine.pddl_type = pddl_type_mongoengine

            pddl_action_mongoengine.pddl_parameters.append(
                pddl_parameter_mongoengine)

            parameter_dict[param_name] = pddl_parameter_mongoengine

        # ACTION CONDIS
        for condi in pddl_dto_action.get_conditions_list():
            pddl_condi_mongoengine = self.__condition_effect_dto_to_mongoengine(
                condi, parameter_dict)

            if(not pddl_condi_mongoengine):
                return None

            pddl_action_mongoengine.conditions.append(pddl_condi_mongoengine)
            pddl_action_mongoengine._pddl_predicates_used.append(
                pddl_condi_mongoengine.pddl_predicate)

        # ACTION EFFECTS
        for effect in pddl_dto_action.get_effects_list():
            pddl_effect_mongoengine = self.__condition_effect_dto_to_mongoengine(
                effect, parameter_dict)

            if(not pddl_effect_mongoengine):
                return None

            pddl_action_mongoengine.effects.append(pddl_effect_mongoengine)
            pddl_action_mongoengine._pddl_predicates_used.append(
                pddl_effect_mongoengine.pddl_predicate)

        return pddl_action_mongoengine

    def _exist_in_mongo(self, pddl_dto_action):

        pddl_action_mongoengine = ppddl_action_mongoengine_model.objects(
            action_name=pddl_dto_action.get_action_name())
        if(pddl_action_mongoengine):
            return True
        return False

    def get(self, action_name):

        pddl_action_mongoengine = ppddl_action_mongoengine_model.objects(
            action_name=action_name)

        # check if action exists
        if(pddl_action_mongoengine):
            pddl_action_mongoengine = pddl_action_mongoengine[0]
            pddl_dto_action = self._mongoengine_to_dto(
                pddl_action_mongoengine)
            return pddl_dto_action

        else:
            return None

    def get_all(self):

        pddl_action_mongoengine = ppddl_action_mongoengine_model.objects
        pddl_dto_action_list = []

        for ele in pddl_action_mongoengine:
            pddl_dto_action = self._mongoengine_to_dto(ele)
            pddl_dto_action_list.append(pddl_dto_action)

        return pddl_dto_action_list

    def save(self, pddl_dto_action):

        if(self._exist_in_mongo(pddl_dto_action)):
            return False

        pddl_action_mongoengine = self._dto_to_mongoengine(
            pddl_dto_action)

        if(pddl_action_mongoengine):
            pddl_action_mongoengine.save()
            return True

        else:
            return False

    def update(self, pddl_dto_action):

        pddl_action_mongoengine = ppddl_action_mongoengine_model.objects(
            action_name=pddl_dto_action.get_action_name())

        # check if action exists
        if(pddl_action_mongoengine):
            new_pddl_action_mongoengine = self._dto_to_mongoengine(
                pddl_dto_action)

            if(new_pddl_action_mongoengine):
                pddl_action_mongoengine = pddl_action_mongoengine[0]
                pddl_action_mongoengine.action_name = new_pddl_action_mongoengine.action_name
                pddl_action_mongoengine.durative = new_pddl_action_mongoengine.durative
                pddl_action_mongoengine.duration = new_pddl_action_mongoengine.duration
                pddl_action_mongoengine.pddl_parameters = new_pddl_action_mongoengine.pddl_parameters
                pddl_action_mongoengine.conditions = new_pddl_action_mongoengine.conditions
                pddl_action_mongoengine.effects = new_pddl_action_mongoengine.effects
                pddl_action_mongoengine.save()
            else:
                return False

            return True

        else:
            return False

    def save_update(self, pddl_dto_action):

        if(self._exist_in_mongo(pddl_dto_action)):
            return self.update(pddl_dto_action)

        else:
            return self.save(pddl_dto_action)

    def delete(self, pddl_dto_action):

        pddl_action_mongoengine = ppddl_action_mongoengine_model.objects(
            action_name=pddl_dto_action.get_action_name())

        # check if action exists
        if(pddl_action_mongoengine):
            pddl_action_mongoengine = pddl_action_mongoengine[0]
            pddl_action_mongoengine.delete()
            return True

    def delete_all(self):
        pddl_dto_action_list = self.get_all()

        for ele in pddl_dto_action_list:
            self.delete(ele)

        return True
