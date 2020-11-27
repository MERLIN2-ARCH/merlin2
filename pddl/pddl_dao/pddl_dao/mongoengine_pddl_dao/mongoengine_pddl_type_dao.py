
""" Mongoengine Pddl Type Dao """

from typing import List

from pddl_dao.pddl_dao_interface import PddlTypeDao
from pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlTypeModel

from pddl_dto import PddlTypeDto


class MongoenginePddlTypeDao(PddlTypeDao, MongoenginePddlDao):
    """ Mongoengine Pddl Type Dao Class """

    def __init__(self, uri: str = None):

        PddlTypeDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

    def _model_to_dto(self, pddl_type_model:
                      PddlTypeModel) -> PddlTypeDto:
        """ convert a Mongoengine pddl type document into a PddlTypeDto

        Args:
            pddl_type_model (PddlTypeModel): Mongoengine pddl type document

        Returns:
            PddlTypeDto: PddlTypeDto
        """

        pddl_type_dto = PddlTypeDto(pddl_type_model.type_name)
        return pddl_type_dto

    def _dto_to_model(self, pddl_type_dto: PddlTypeDto) -> PddlTypeModel:
        """ convert a PddlTypeDto into a Mongoengine pddl type document

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            Document: Mongoengine pddl type document
        """

        pddl_type_model = PddlTypeModel()
        pddl_type_model.type_name = pddl_type_dto.get_type_name()
        return pddl_type_model

    def _exist_in_mongo(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ check if PddlTypeDto exists

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            bool: PddlTypeDto exists?
        """

        if self._get_model(pddl_type_dto):
            return True
        return False

    def _get_model(self, pddl_type_dao: PddlTypeDto) -> PddlTypeModel:
        """ get the Mongoengine pddl type document corresponding to a give PddlTypeDto

        Args:
            pddl_type_dao (PddlTypeDto): PddlTypeDto

        Returns:
            Document: Mongoengine pddl type document
        """

        pddl_type_model = PddlTypeModel.objects(
            type_name=pddl_type_dao.get_type_name())

        if not pddl_type_model:
            return None

        return pddl_type_model[0]

    def get(self, type_name: str) -> PddlTypeDto:
        """ get a PddlTypeDto with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlTypeDto: PddlTypeDto of the pddl type name
        """

        pddl_type_model = PddlTypeModel.objects(
            type_name=type_name)

        if pddl_type_model:
            pddl_type_model = pddl_type_model[0]
            return self._model_to_dto(pddl_type_model)

        return None

    def get_all(self) -> List[PddlTypeDto]:
        """ get all PddlTypeDto

        Returns:
            List[PddlTypeDto]: list of all PddlTypeDto
        """

        pddl_type_model = PddlTypeModel.objects
        pddl_type_dto_list = []

        for ele in pddl_type_model:
            pddl_type_dto = self._model_to_dto(ele)
            pddl_type_dto_list.append(pddl_type_dto)

        return pddl_type_dto_list

    def _save(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save a PddlTypeDto
            if the PddlTypeDto is already saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_type_dto):
            return False

        pddl_type_model = self._dto_to_model(pddl_type_dto)
        pddl_type_model.save()
        return True

    def _update(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ update a PddlTypeDto
            if the PddlTypeDto is not saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to update

        Returns:
            bool: succeed
        """

        pddl_type_model = self._get_model(pddl_type_dto)

        # check if type exists
        if pddl_type_model:
            pddl_type_model.type_name = pddl_type_dto.get_type_name()
            pddl_type_model.save()
            return True

        return False

    def save(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save or update a PddlTypeDto
            if the PddlTypeDto is not saved it will be saved, else it will be updated

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_type_dto):
            return self._update(pddl_type_dto)

        return self._save(pddl_type_dto)

    def delete(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ delete a PddlTypeDto
            if the PddlTypeDto is not saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to delete

        Returns:
            bool: succeed
        """

        pddl_type_model = self._get_model(pddl_type_dto)

        # check if type exists
        if pddl_type_model:
            pddl_type_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl types

        Returns:
            bool: succeed
        """

        pddl_type_dto_list = self.get_all()

        for ele in pddl_type_dto_list:
            self.delete(ele)

        return True
