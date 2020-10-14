
""" Mongoengine Pddl Predicate Dao """

from typing import List

from pddl_dao.pddl_dao_interface.pddl_predicate_dao import PddlPredicateDao
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlPredicateModel

from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dto.pddl_type_dto import PddlTypeDto

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_type_dao import MongoenginePddlTypeDao


class MongoenginePddlPredicateDao(PddlPredicateDao, MongoenginePddlDao):
    """ Mongoengine Pddl Predicate Dao Class """

    def __init__(self, uri: str = None):

        PddlPredicateDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._me_pddl_type_dao = MongoenginePddlTypeDao(uri)

    def _model_to_dto(self,
                      pddl_predicate_model: PddlPredicateModel) -> PddlPredicateDto:
        """ convert a Mongoengine pddl predicate document into a PddlPredicateDto

        Args:
            pddl_predicate_model (PddlPredicateModel): Mongoengine pddl predicate document

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        pddl_dto_type_list = []

        for pddl_type_model in pddl_predicate_model.pddl_types:
            pddl_type_dto = PddlTypeDto(pddl_type_model.type_name)
            pddl_dto_type_list.append(pddl_type_dto)

        pddl_predicate_dto = PddlPredicateDto(
            pddl_predicate_model.predicate_name, pddl_dto_type_list)

        return pddl_predicate_dto

    def _dto_to_model(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicateModel:
        """ convert a PddlPredicateDto into a Mongoengine pddl predicate document

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            Document: Mongoengine pddl predicate document
        """

        pddl_predicate_model = PddlPredicateModel()

        pddl_predicate_model.predicate_name = pddl_predicate_dto.get_predicate_name()

        for pddl_type_dto in pddl_predicate_dto.get_pddl_types_list():

            pddl_type_model = self._me_pddl_type_dao._get_model(
                pddl_type_dto)

            # check if type exist
            if not pddl_type_model:
                return None

            pddl_predicate_model.pddl_types.append(pddl_type_model)

        return pddl_predicate_model

    def _exist_in_mongo(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ check if PddlPredicateDto exists

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: PddlPredicateDto exists?
        """

        if self._get_model(pddl_predicate_dto):
            return True
        return False

    def _get_model(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicateModel:
        """ get the Mongoengine pddl predicate document corresponding to a give PddlPredicateDto

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            Document: Mongoengine pddl predicate document
        """

        pddl_predicate_model = PddlPredicateModel.objects(
            predicate_name=pddl_predicate_dto.get_predicate_name())

        if not pddl_predicate_model:
            return None

        return pddl_predicate_model[0]

    def get(self, predicate_name: str) -> PddlPredicateDto:
        """ get a PddlPredicateDto with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto of the pddl predicate name
        """

        pddl_predicate_model = PddlPredicateModel.objects(
            predicate_name=predicate_name)

        # check if predicate exist
        if pddl_predicate_model:
            pddl_predicate_model = pddl_predicate_model[0]
            pddl_predicate_dto = self._model_to_dto(
                pddl_predicate_model)
            return pddl_predicate_dto

        return None

    def get_all(self) -> List[PddlPredicateDto]:
        """ get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of all PddlPredicateDto
        """
        pddl_predicate_model = PddlPredicateModel.objects
        pddl_dto_predicate_list = []

        for ele in pddl_predicate_model:
            pddl_predicate_dto = self._model_to_dto(ele)
            pddl_dto_predicate_list.append(pddl_predicate_dto)

        return pddl_dto_predicate_list

    def _save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save a PddlPredicateDto
            if the PddlPredicateDto is already saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_predicate_dto):
            return False

        # propagating saving
        for pddl_type_dto in pddl_predicate_dto.get_pddl_types_list():
            result = self._me_pddl_type_dao.save(pddl_type_dto)
            if not result:
                return False

        pddl_predicate_model = self._dto_to_model(
            pddl_predicate_dto)

        if pddl_predicate_model:
            pddl_predicate_model.save()
            return True

        return False

    def _update(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ update a PddlPredicateDto
             if the PddlPredicateDto is not saved return False, else return True

         Args:
             pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

         Returns:
             bool: succeed
         """

        pddl_predicate_model = self._get_model(pddl_predicate_dto)

        # check if predicate exists
        if pddl_predicate_model:
            new_pddl_predicate_model = self._dto_to_model(
                pddl_predicate_dto)

            if new_pddl_predicate_model:
                pddl_predicate_model.predicate_name = new_pddl_predicate_model.predicate_name
                pddl_predicate_model.pddl_types = new_pddl_predicate_model.pddl_types
                pddl_predicate_model.save()
            else:
                return False

            return True

        return False

    def save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save or update a PddlPredicateDto
            if the PddlPredicateDto is not saved it will be saved, else it will be updated

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_predicate_dto):
            return self._update(pddl_predicate_dto)

        return self._save(pddl_predicate_dto)

    def delete(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ delete a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to delete

        Returns:
            bool: succeed
        """

        pddl_predicate_model = self._get_model(pddl_predicate_dto)

        # check if predicate exists
        if pddl_predicate_model:
            pddl_predicate_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl predicates

        Returns:
            bool: succeed
        """

        pddl_dto_predicate_list = self.get_all()

        for ele in pddl_dto_predicate_list:
            self.delete(ele)

        return True
