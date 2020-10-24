
""" Mongoengine Pddl Dao Action """


from typing import List

from pddl_dao.pddl_dao_interface.pddl_action_dao import PddlActionDao
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlActionModel
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlConditionEffectModel
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlParameterModel

from pddl_dto.pddl_type_dto import PddlTypeDto
from pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dto.pddl_object_dto import PddlObjectDto
from pddl_dto.pddl_action_dto import PddlActionDto

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_proposition_dao import(
    MongoenginePddlPropositionDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_type_dao import(
    MongoenginePddlTypeDao
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_predicate_dao import (
    MongoenginePddlPredicateDao
)


class MongoenginePddlActionDao(PddlActionDao, MongoenginePddlDao):
    """ Mongoengine Pddl Dao Action Class """

    def __init__(self, uri: str = None):

        PddlActionDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._me_pddl_type_dao = MongoenginePddlTypeDao(uri)
        self._me_pddl_predicate_dao = MongoenginePddlPredicateDao(uri)

    def __condition_effect_model_to_dto(self,
                                        pddl_condition_effect_model: PddlConditionEffectModel,
                                        parameter_dict: dict) -> PddlConditionEffectDto:
        """ convert a Mongoengine pddl condition/effect document into a PddlConditionEffectDto

        Args:
            pddl_condition_effect_model
            (PddlConditionEffectModel): Mongoengine pddl condition/effect document

        Returns:
            PddlConditionEffectDto: PddlConditionEffectDto
        """

        pddl_predicate_dao = MongoenginePddlPredicateDao(self.get_uri())

        pddl_dto_precicate = pddl_predicate_dao.get(
            pddl_condition_effect_model.pddl_predicate.predicate_name)

        pddl_dto_condition_effect = PddlConditionEffectDto(
            pddl_dto_precicate,
            is_negative=pddl_condition_effect_model.is_negative,
            time=pddl_condition_effect_model.time)

        pddl_dto_objects_list = []

        for pddl_parameter_model in pddl_condition_effect_model.pddl_parameters:
            pddl_object_dto = parameter_dict[pddl_parameter_model.parameter_name]
            pddl_dto_objects_list.append(pddl_object_dto)

        pddl_dto_condition_effect.set_pddl_objects_list(pddl_dto_objects_list)

        return pddl_dto_condition_effect

    def _model_to_dto(self, pddl_action_model: PddlActionModel) -> PddlActionDto:
        """ convert a Mongoengine pddl action document into a PddlActionDto

        Args:
            pddl_action_model (PddlActionModel): Mongoengine pddl action document

        Returns:
            PddlActionDto: PddlActionDto
        """

        pddl_action_dto = PddlActionDto(
            pddl_action_model.action_name)
        pddl_action_dto.set_duration(pddl_action_model.duration)
        pddl_action_dto.set_durative(pddl_action_model.durative)

        parameters_list = []
        conditions_list = []
        effects_list = []
        parameter_dict = {}

        # ACTION PARAMS
        for pddl_parameter_model in pddl_action_model.pddl_parameters:
            pddl_type_dto = PddlTypeDto(
                pddl_parameter_model.pddl_type.type_name)
            pddl_object_dto = PddlObjectDto(
                pddl_type_dto, pddl_parameter_model.parameter_name)
            parameter_dict[pddl_parameter_model.parameter_name] = pddl_object_dto
            parameters_list.append(pddl_object_dto)

        # ACTION CONDIS
        for pddl_condition_model in pddl_action_model.conditions:
            pddl_dto_condition_effect = self.__condition_effect_model_to_dto(
                pddl_condition_model, parameter_dict)
            conditions_list.append(pddl_dto_condition_effect)

        # ACTION EFFECTS
        for pddl_effect_model in pddl_action_model.effects:
            pddl_dto_condition_effect = self.__condition_effect_model_to_dto(
                pddl_effect_model, parameter_dict)
            effects_list.append(pddl_dto_condition_effect)

        pddl_action_dto.set_parameters_list(parameters_list)
        pddl_action_dto.set_conditions_list(conditions_list)
        pddl_action_dto.set_effects_list(effects_list)

        return pddl_action_dto

    def __condition_effect_dto_to_model(self,
                                        pddl_dto_condition_effect: PddlConditionEffectDto,
                                        parameter_dict: dict) -> PddlConditionEffectModel:
        """ convert a PddlConditionEffectDto into a Mongoengine pddl condition/effect document

        Args:
            pddl_dto_condition_effect (PddlConditionEffectDto): PddlConditionEffectDto

        Returns:
            Document: Mongoengine pddl condition/effect document
        """

        pddl_predicate_model = self._me_pddl_predicate_dao._get_model(
            pddl_dto_condition_effect.get_pddl_predicate())

        # check if predicate exists
        if not pddl_predicate_model:
            return None

        pddl_condition_model = PddlConditionEffectModel()
        pddl_condition_model.pddl_predicate = pddl_predicate_model
        pddl_condition_model.time = pddl_dto_condition_effect.get_time()
        pddl_condition_model.is_negative = pddl_dto_condition_effect.get_is_negative()

        for param in pddl_dto_condition_effect.get_pddl_objects_list():

            pddl_condition_model.pddl_parameters.append(
                parameter_dict[param.get_object_name()])

        return pddl_condition_model

    def _dto_to_model(self, pddl_action_dto: PddlActionDto) -> PddlActionModel:
        """ convert a PddlActionDto into a Mongoengine pddl action document

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            Document: Mongoengine pddl action document
        """

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

            pddl_parameter_model = PddlParameterModel()
            pddl_parameter_model.parameter_name = param_name
            pddl_parameter_model.pddl_type = pddl_type_model

            pddl_action_model.pddl_parameters.append(
                pddl_parameter_model)

            parameter_dict[param_name] = pddl_parameter_model

        # ACTION CONDIS
        for pddl_condition_dto in pddl_action_dto.get_conditions_list():
            pddl_condition_model = self.__condition_effect_dto_to_model(
                pddl_condition_dto, parameter_dict)

            if not pddl_condition_model:
                return None

            pddl_action_model.conditions.append(pddl_condition_model)
            pddl_action_model._pddl_predicates_used.append(
                pddl_condition_model.pddl_predicate)

        # ACTION EFFECTS
        for pddl_effect_dto in pddl_action_dto.get_effects_list():
            pddl_effect_model = self.__condition_effect_dto_to_model(
                pddl_effect_dto, parameter_dict)

            if not pddl_effect_model:
                return None

            pddl_action_model.effects.append(pddl_effect_model)
            pddl_action_model._pddl_predicates_used.append(
                pddl_effect_model.pddl_predicate)

        return pddl_action_model

    def _exist_in_mongo(self, pddl_action_dto: PddlActionDto) -> bool:
        """ check if PddlActionDto exists

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            bool: PddlActionDto exists?
        """

        if self._get_model(pddl_action_dto):
            return True
        return False

    def _get_model(self, pddl_action_dto: PddlActionDto) -> PddlActionModel:
        """ get the Mongoengine pddl action document corresponding to a give PddlActionDto

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            Document: Mongoengine pddl action document
        """

        pddl_action_model = PddlActionModel.objects(
            action_name=pddl_action_dto.get_action_name())
        if not pddl_action_model:
            return None
        return pddl_action_model[0]

    def _check_pddl_condition_efect_dto(self,
                                        pddl_condition_effect_dto: PddlConditionEffectDto,
                                        pddl_parameter_dtos: PddlObjectDto) -> bool:
        """ check if the types of the objects of a pddl codition/effect dto are
            the same as the types of its predicate and if that objects are action parameters

        Args:
            pddl_condition_effect_dto
            (PddlConditionEffectDto): PddlConditionEffectDto

            pddl_parameter_dtos
            (PddlObjectDto): PddlObjectDto

        Returns:
            bool: condition/effect is correct?
        """

       # check if proposition is correct
        if(len(pddl_condition_effect_dto.get_pddl_objects_list()) !=
           len(pddl_condition_effect_dto.get_pddl_predicate().get_pddl_types_list())):
            return False

        pddl_object_dtos = pddl_condition_effect_dto.get_pddl_objects_list()
        pddl_type_dtos = pddl_condition_effect_dto.get_pddl_predicate().get_pddl_types_list()

        for pddl_object_dto, pddl_type_dto in zip(pddl_object_dtos, pddl_type_dtos):

            # check if condition/effect object type is a parameter
            if not pddl_object_dto in pddl_parameter_dtos:
                return False

            # check if condition/effect object type is correct
            if pddl_object_dto.get_pddl_type() != pddl_type_dto:
                return False

        return True

    def _check_pddl_action_dto(self, pddl_action_dto: PddlActionDto) -> bool:
        """ check if a PddlActionDto is correct:
            condition and effect must be correct (same as proposition)

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to check

        Returns:
            bool: is PddlActionDto correct?
        """

        for pddl_condi_effect_dto in (pddl_action_dto.get_conditions_list() +
                                      pddl_action_dto.get_effects_list()):
            if(not pddl_action_dto.get_durative() and pddl_condi_effect_dto.get_time()):
                return False
            elif(pddl_action_dto.get_durative() and not pddl_condi_effect_dto.get_time()):
                return False

            if not self._check_pddl_condition_efect_dto(pddl_condi_effect_dto,
                                                        pddl_action_dto.get_parameters_list()):
                return False

        return True

    def _check_pddl_condition_efect_model(self,
                                          pddl_condition_effect_model: PddlConditionEffectModel,
                                          pddl_parameter_models: PddlParameterModel) -> bool:
        """ check if the types of the objects of a pddl codition/effect model are
            the same as the types of its predicate and if that objects are action parameters

        Args:
            pddl_condition_effect_model
            (PddlConditionEffectModel): Mongoengine pddl condition/effect document

            pddl_parameter_models
            (PddlParameterModel): Mongoengine pddl parameter documents

        Returns:
            bool: condition/effect is correct?
        """

        # check if proposition is correct
        if(len(pddl_condition_effect_model.pddl_parameters) !=
           len(pddl_condition_effect_model.pddl_predicate.pddl_types)):
            return False

        pddl_parameter_models = pddl_condition_effect_model.pddl_parameters
        pddl_type_models = pddl_condition_effect_model.pddl_predicate.pddl_types

        for pddl_parameter_model, pddl_type_model in zip(pddl_parameter_models, pddl_type_models):

            if not pddl_parameter_model in pddl_parameter_models:
                return False

            # check if proposition is correct
            if pddl_parameter_model.pddl_type.type_name != pddl_type_model.type_name:
                return False

        return True

    def _check_pddl_action_model(self, pddl_action_model: PddlActionModel) -> bool:
        """ check if a PddlActionDto is correct:
            condition and effect must be correct (same as proposition)

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to check

        Returns:
            bool: is PddlActionDto correct?
        """

        for pddl_condi_effect_model in (pddl_action_model.conditions +
                                        pddl_action_model.effects):
            if(not pddl_action_model.durative and pddl_condi_effect_model.time):
                return False
            elif(pddl_action_model.durative and not pddl_condi_effect_model.time):
                return False

            if not self._check_pddl_condition_efect_model(pddl_condi_effect_model,
                                                          pddl_action_model.pddl_parameters):
                return False

        return True

    def get(self, action_name: str) -> PddlActionDto:
        """ get a PddlActionDto with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlActionDto: PddlActionDto of the pddl action name
        """

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

        return None

    def get_all(self) -> List[PddlActionDto]:
        """ get all PddlActionDto

        Returns:
            List[PddlActionDto]: list of all PddlActionDto
        """

        pddl_action_model = PddlActionModel.objects
        pddl_dto_action_list = []

        for ele in pddl_action_model:
            if self._check_pddl_action_model(ele):
                pddl_action_dto = self._model_to_dto(ele)
                pddl_dto_action_list.append(pddl_action_dto)

        return pddl_dto_action_list

    def _save(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save a PddlActionDto
            if the PddlActionDto is already saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save

        Returns:
            bool: succeed
        """

        if not self._check_pddl_action_dto(pddl_action_dto):
            return False

       # propagating saving
        for pddl_type_dto in pddl_action_dto.get_parameters_list():
            result = self._me_pddl_type_dao.save(
                pddl_type_dto.get_pddl_type())
            if not result:
                return False
        for pddl_condition_dto in pddl_action_dto.get_conditions_list():
            result = self._me_pddl_predicate_dao.save(
                pddl_condition_dto.get_pddl_predicate())
            if not result:
                return False
        for pddl_effect_dto in pddl_action_dto.get_effects_list():
            result = self._me_pddl_predicate_dao.save(
                pddl_effect_dto.get_pddl_predicate())
            if not result:
                return False

        if self._exist_in_mongo(pddl_action_dto):
            return False

        pddl_action_model = self._dto_to_model(
            pddl_action_dto)

        if pddl_action_model:
            pddl_action_model.save()
            return True

        return False

    def _update(self, pddl_action_dto: PddlActionDto) -> bool:
        """ update a PddlActionDto
            if the PddlActionDto is not saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to update

        Returns:
            bool: succeed
        """

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

        return False

    def save(self, pddl_action_dto: PddlActionDto) -> bool:

        if self._exist_in_mongo(pddl_action_dto):
            return self._update(pddl_action_dto)

        return self._save(pddl_action_dto)

    def delete(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save or update a PddlActionDto
            if the PddlActionDto is not saved it will be saved, else it will be updated

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save or update

        Returns:
            bool: succeed
        """

        pddl_action_model = self._get_model(pddl_action_dto)

        # check if action exists
        if pddl_action_model:
            pddl_action_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl actions

        Returns:
            bool: succeed
        """

        pddl_dto_action_list = self.get_all()

        for ele in pddl_dto_action_list:
            self.delete(ele)

        return True
