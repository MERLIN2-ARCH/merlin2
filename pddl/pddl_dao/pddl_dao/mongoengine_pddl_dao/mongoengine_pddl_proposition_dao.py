
""" Mongoengine Pddl Proposition Dao """

from typing import List

from pddl_dao.pddl_dao_interface import PddlPropositionDao
from pddl_dao.mongoengine_pddl_dao import (
    MongoenginePddlDao,
    MongoenginePddlPredicateDao,
    MongoenginePddlObjectDao
)

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import (
    PddlPropositionModel,
    PddlPredicateModel
)

from pddl_dto import PddlPropositionDto


class MongoenginePddlPropositionDao(PddlPropositionDao, MongoenginePddlDao):
    """ Mongoengine Pddl Proposition Dao Class """

    def __init__(self, uri: str = None, connect: bool = True):

        PddlPropositionDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        if connect:
            self.connect()

        self._me_pddl_object_dao = MongoenginePddlObjectDao(uri, connect=False)
        self._me_pddl_predicate_dao = MongoenginePddlPredicateDao(
            uri, connect=False)

    def _check_pddl_proposition_model(self,
                                      pddl_proposition_model: PddlPropositionModel) -> bool:
        """ check if the types of the objects of a pddl proposition model are
            the same as the types of its predicate

        Args:
            pddl_proposition_model (PddlPropositionModel): Mongoengine pddl proposition document

        Returns:
            bool: poposition is correct?
        """

        # check if proposition is correct
        if(len(pddl_proposition_model.pddl_objects) !=
           len(pddl_proposition_model.pddl_predicate.pddl_types)):
            return False

        pddl_object_models = pddl_proposition_model.pddl_objects
        pddl_type_models = pddl_proposition_model.pddl_predicate.pddl_types

        for pddl_object_model, pddl_type_model in zip(pddl_object_models, pddl_type_models):

            # check if proposition is correct
            if pddl_object_model.pddl_type.type_name != pddl_type_model.type_name:
                return False

        return True

    def _model_to_dto(self,
                      pddl_proposition_model: PddlPropositionModel) -> PddlPropositionDto:
        """ convert a Mongoengine pddl type document into a PddlPropositionDto

        Args:
            pddl_proposition_model (PddlPropositionModel): Mongoengine pddl proposition document

        Returns:
            PddlPropositionDto: PddlPropositionDto
        """

        pddl_object_list = []

        pddl_predicate_dto = self._me_pddl_predicate_dao._model_to_dto(
            pddl_proposition_model.pddl_predicate)

        for pddl_object_model in pddl_proposition_model.pddl_objects:

            pddl_object_dto = self._me_pddl_object_dao._model_to_dto(
                pddl_object_model)

            # check if object exist
            if not pddl_object_dto:
                return None

            pddl_object_list.append(pddl_object_dto)

        pddl_proposition_dto = PddlPropositionDto(
            pddl_predicate_dto, pddl_object_list)

        pddl_proposition_dto.set_is_goal(
            pddl_proposition_model.is_goal)

        return pddl_proposition_dto

    def _check_pddl_proposition_dto(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ check if the types of the objects of a pddl proposition dto are
            the same as the types of its predicate

        Args:
            pddl_proposition_dto (PddlPropositionDto): pddl proposition dto

        Returns:
            bool: poposition is correct?
        """

       # check if proposition is correct
        if(len(pddl_proposition_dto.get_pddl_objects_list()) !=
           len(pddl_proposition_dto.get_pddl_predicate().get_pddl_types_list())):
            return False

        pddl_object_dtos = pddl_proposition_dto.get_pddl_objects_list()
        pddl_type_dtos = pddl_proposition_dto.get_pddl_predicate().get_pddl_types_list()

        for pddl_object_dto, pddl_type_dto in zip(pddl_object_dtos, pddl_type_dtos):

            # check if proposition is correct
            if pddl_object_dto.get_pddl_type() != pddl_type_dto:
                return False

        return True

    def _dto_to_model(self, pddl_proposition_dto: PddlPropositionDto) -> PddlPropositionModel:
        """ convert a PddlPropositionDto into a Mongoengine pddl propostion document

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            Document: Mongoengine pddl propostion document
        """

        pddl_proposition_model = PddlPropositionModel()

        # predicate model
        pddl_predicate_model = self._me_pddl_predicate_dao._dto_to_model(
            pddl_proposition_dto.get_pddl_predicate())

        pddl_proposition_model.pddl_predicate = pddl_predicate_model

        # is goal
        pddl_proposition_model.is_goal = pddl_proposition_dto.get_is_goal()

        # objects models
        for pddl_object_dto in pddl_proposition_dto.get_pddl_objects_list():

            pddl_object_model = self._me_pddl_object_dao._dto_to_model(
                pddl_object_dto)

            pddl_proposition_model.pddl_objects.append(
                pddl_object_model)

        return pddl_proposition_model

    def _exist_in_mongo(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ check if PddlPropositionDto exists

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: PddlPropositionDto exists?
        """

        if self._get_model(pddl_proposition_dto):
            return True
        return False

    def _get_model(self, pddl_proposition_dto: PddlPropositionDto) -> PddlPropositionModel:
        """ get the Mongoengine pddl proposition document corresponding to a give PddlPropositionDto

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            Document: Mongoengine pddl proposition document
        """

        pddl_objects_dto_list = pddl_proposition_dto.get_pddl_objects_list()

        pddl_objects_list = []
        for pddl_object_dto in pddl_objects_dto_list:
            pddl_objects_list.append(pddl_object_dto.get_object_name())

        # getting proposition
        pddl_proposition_model = PddlPropositionModel.objects(
            pddl_predicate=pddl_proposition_dto.get_pddl_predicate().get_predicate_name(),
            pddl_objects=pddl_objects_list)

        # check if proposition exist
        if not pddl_proposition_model:
            return None

        return pddl_proposition_model[0]

    def get_by_predicate(self, predicate_name: str) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto with a given pddl predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        pddl_proposition_model = PddlPropositionModel.objects(
            pddl_predicate=predicate_name)

        pddl_dto_proposition_list = []

        for ele in pddl_proposition_model:
            if self._check_pddl_proposition_model(ele):
                pddl_dto_proposition_list.append(self._model_to_dto(ele))

        return pddl_dto_proposition_list

    def _get_all(self, is_goal: bool = None) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto
            is_goal == None -> get all proposition
            is_goal == True -> gel all goals
            is_goal == False -> getl no goals

        Args:
            is_goal (bool, optional): get all, all goals, all no goals?. Defaults to None.

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        if(is_goal is None):
            pddl_proposition_model = PddlPropositionModel.objects()
        else:
            pddl_proposition_model = PddlPropositionModel.objects(
                is_goal=is_goal)

        pddl_dto_proposition_list = []

        for ele in pddl_proposition_model:
            if self._check_pddl_proposition_model(ele):
                pddl_dto_proposition_list.append(self._model_to_dto(ele))

        return pddl_dto_proposition_list

    def get_goals(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto that are goals

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self._get_all(is_goal=True)

    def get_no_goals(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto that are not goals

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self._get_all(is_goal=False)

    def get_all(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self._get_all()

    def _save(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save a PddlPropositionDto
            if the PddlPropositionDto is already saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_proposition_dto):
            return False

        if not self._check_pddl_proposition_dto(pddl_proposition_dto):
            return False

       # propagating saving
        for pddl_object_dto in pddl_proposition_dto.get_pddl_objects_list():
            result = self._me_pddl_object_dao.save(pddl_object_dto)
            if not result:
                return False

        # saving
        pddl_proposition_model = self._dto_to_model(
            pddl_proposition_dto)

        if pddl_proposition_model:
            pddl_proposition_model.save(cascade=True)
            return True

        return False

    def _update(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ update a PddlPropositionDto
            if the PddlPropositionDto is not saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to update

        Returns:
            bool: succeed
        """

        if not self._check_pddl_proposition_dto(pddl_proposition_dto):
            return False

        pddl_proposition_model = self._get_model(
            pddl_proposition_dto)

        # check if proposition exists
        if pddl_proposition_model:
            new_pddl_proposition_model = self._dto_to_model(
                pddl_proposition_dto)
            pddl_proposition_model.pddl_predicate = new_pddl_proposition_model.pddl_predicate
            pddl_proposition_model.pddl_objects = new_pddl_proposition_model.pddl_objects
            pddl_proposition_model.is_goal = new_pddl_proposition_model.is_goal
            pddl_proposition_model.save()

            return True

        return False

    def save(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save or update a PddlPropositionDto
            if the PddlPropositionDto is not saved it will be saved, else it will be updated

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_proposition_dto):
            return self._update(pddl_proposition_dto)

        return self._save(pddl_proposition_dto)

    def delete(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ delete a PddlPropositionDto
            if the PddlPropositionDto is not saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to delete

        Returns:
            bool: succeed
        """

        pddl_proposition_model = self._get_model(
            pddl_proposition_dto)

        # check if proposition exists
        if pddl_proposition_model:
            pddl_proposition_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl propositions

        Returns:
            bool: succeed
        """

        pddl_dto_proposition_list = self.get_all()

        for ele in pddl_dto_proposition_list:
            self.delete(ele)

        return True
