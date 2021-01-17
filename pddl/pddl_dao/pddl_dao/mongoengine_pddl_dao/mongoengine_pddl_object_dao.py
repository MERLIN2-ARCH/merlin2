
""" Mongoengine Pddl Object Dao """

from typing import List

from pddl_dao.pddl_dao_interface import PddlObjectDao
from pddl_dao.mongoengine_pddl_dao import (
    MongoenginePddlDao,
    MongoenginePddlTypeDao
)

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import PddlObjectModel


from pddl_dto import (
    PddlObjectDto,
    PddlTypeDto
)


class MongoenginePddlObjectDao(PddlObjectDao, MongoenginePddlDao):
    """ Mongoengine Pddl Object Dao Class """

    def __init__(self, uri: str = None, connect: bool = True):

        PddlObjectDao.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        if connect:
            self.connect()

        self._me_pddl_type_dao = MongoenginePddlTypeDao(uri, connect=False)

    def _model_to_dto(self, pddl_object_model: PddlObjectModel) -> PddlObjectDto:
        """ convert a Mongoengine pddl object document into a PddlObjectDto

        Args:
            pddl_object_model (PddlObjectModel): Mongoengine pddl object document

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        pddl_type_dto = PddlTypeDto(
            pddl_object_model.pddl_type.type_name)

        pddl_object_dto = PddlObjectDto(pddl_type_dto,
                                        pddl_object_model.object_name)

        return pddl_object_dto

    def _dto_to_model(self, pddl_object_dto: PddlObjectDto) -> PddlObjectModel:
        """ convert a PddlObjectDto into a Mongoengine pddl object document

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            Document: Mongoengine pddl object document
        """

        pddl_type_model = self._me_pddl_type_dao._dto_to_model(
            pddl_object_dto.get_pddl_type())

        pddl_object_model = PddlObjectModel()

        pddl_object_model.object_name = pddl_object_dto.get_object_name()

        pddl_object_model.pddl_type = pddl_type_model

        return pddl_object_model

    def _exist_in_mongo(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ check if PddlObjectDto exists

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: PddlObjectDto exists?
        """

        if self._get_model(pddl_object_dto):
            return True
        return False

    def _get_model(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ get the Mongoengine pddl object document corresponding to a give PddlObjectDto

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            Document: Mongoengine pddl object document
        """

        pddl_object_model = PddlObjectModel.objects(
            object_name=pddl_object_dto.get_object_name())

        if not pddl_object_model:
            return None

        return pddl_object_model[0]

    def get(self, object_name: str) -> PddlObjectDto:
        """ get a PddlObjectDto with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlObjectDto: PddlObjectDto of the pddl object name
        """

        pddl_object_model = PddlObjectModel.objects(
            object_name=object_name)

        # check if object exists
        if pddl_object_model:
            pddl_object_model = pddl_object_model[0]
            pddl_object_dto = self._model_to_dto(
                pddl_object_model)
            return pddl_object_dto

        return None

    def get_all(self) -> List[PddlObjectDto]:
        """ get all PddlObjectDto

        Returns:
            List[PddlObjectDto]: list of all PddlObjectDto
        """

        pddl_object_model = PddlObjectModel.objects
        pddl_dto_object_list = []

        for ele in pddl_object_model:
            pddl_object_dto = self._model_to_dto(ele)
            pddl_dto_object_list.append(pddl_object_dto)

        return pddl_dto_object_list

    def _save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save a PddlObjectDto
            if the PddlObjectDto is already saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_object_dto):
            return False

        pddl_object_model = self._dto_to_model(
            pddl_object_dto)

        if pddl_object_model:
            pddl_object_model.save(cascade=True)
            return True

        return False

    def _update(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ update a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to update

        Returns:
            bool: succeed
        """

        pddl_object_model = self._get_model(pddl_object_dto)

        # check if object exists
        if pddl_object_model:
            new_pddl_object_model = self._dto_to_model(
                pddl_object_dto)

            pddl_object_model.object_name = new_pddl_object_model.object_name
            pddl_object_model.pddl_type = new_pddl_object_model.pddl_type
            pddl_object_model.save()

            return True

        return False

    def save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save or update a PddlObjectDto
            if the PddlObjectDto is not saved it will be saved, else it will be updated

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_object_dto):
            return self._update(pddl_object_dto)

        return self._save(pddl_object_dto)

    def delete(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ delete a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to delete

        Returns:
            bool: succeed
        """

        pddl_object_model = self._get_model(pddl_object_dto)

        # check if object exists
        if pddl_object_model:
            pddl_object_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl objects

        Returns:
            bool: succeed
        """

        pddl_dto_object_list = self.get_all()

        for ele in pddl_dto_object_list:
            self.delete(ele)

        return True
