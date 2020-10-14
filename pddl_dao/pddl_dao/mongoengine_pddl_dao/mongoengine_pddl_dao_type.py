
""" Mongoengine Pddl Dao Type """

from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_type import PddlDaoType
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import (
    PddlTypeModel as mongoengine_pddl_type_model
)

from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class MongoenginePddlDaoType(PddlDaoType, MongoenginePddlDao):
    """ Mongoengine Pddl Dao Type Class """

    def __init__(self, uri: str = None):

        PddlDaoType.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

    def _mongoengine_to_dto(self, mongoengine_pddl_type:
                            mongoengine_pddl_type_model) -> PddlDtoType:
        """ convert a Mongoengine pddl type document into a PddlDtoType

        Args:
            mongoengine_pddl_type (mongoengine_pddl_type_model): Mongoengine pddl type document

        Returns:
            PddlDtoType: PddlDtoType
        """

        pddl_dto_type = PddlDtoType(mongoengine_pddl_type.type_name)
        return pddl_dto_type

    def _dto_to_mongoengine(self, pddl_dto_type: PddlDtoType) -> mongoengine_pddl_type_model:
        """ convert a PddlDtoType into a Mongoengine pddl type document

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType

        Returns:
            Document: Mongoengine pddl type document
        """

        mongoengine_pddl_type = mongoengine_pddl_type_model()
        mongoengine_pddl_type.type_name = pddl_dto_type.get_type_name()
        return mongoengine_pddl_type

    def _exist_in_mongo(self, pddl_dto_type: PddlDtoType) -> bool:
        """ check if PddlDtoType exists

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType

        Returns:
            bool: PddlDtoType exists?
        """

        if self._get_mongoengine(pddl_dto_type):
            return True
        return False

    def _get_mongoengine(self, pddl_dao_type: PddlDtoType) -> mongoengine_pddl_type_model:
        """ get the Mongoengine pddl type document corresponding to a give PddlDtoType

        Args:
            pddl_dao_type (PddlDtoType): PddlDtoType

        Returns:
            Document: Mongoengine pddl type document
        """

        mongoengine_pddl_type = mongoengine_pddl_type_model.objects(
            type_name=pddl_dao_type.get_type_name())

        if not mongoengine_pddl_type:
            return None

        return mongoengine_pddl_type[0]

    def get(self, type_name: str) -> PddlDtoType:
        """ get a PddlDtoType with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlDtoType: PddlDtoType of the pddl type name
        """

        mongoengine_pddl_type = mongoengine_pddl_type_model.objects(
            type_name=type_name)

        if mongoengine_pddl_type:
            mongoengine_pddl_type = mongoengine_pddl_type[0]
            return self._mongoengine_to_dto(mongoengine_pddl_type)

        else:
            return None

    def get_all(self) -> List[PddlDtoType]:
        """ get all PddlDtoType

        Returns:
            List[PddlDtoType]: list of all PddlDtoType
        """

        mongoengine_pddl_type = mongoengine_pddl_type_model.objects
        pddl_dto_type_list = []

        for ele in mongoengine_pddl_type:
            pddl_dto_type = self._mongoengine_to_dto(ele)
            pddl_dto_type_list.append(pddl_dto_type)

        return pddl_dto_type_list

    def _save(self, pddl_dto_type: PddlDtoType) -> bool:
        """ save a PddlDtoType
            if the PddlDtoType is already saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_type):
            return False

        mongoengine_pddl_type = self._dto_to_mongoengine(pddl_dto_type)
        mongoengine_pddl_type.save()
        return True

    def _update(self, pddl_dto_type: PddlDtoType) -> bool:
        """ update a PddlDtoType
            if the PddlDtoType is not saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to update

        Returns:
            bool: succeed
        """

        mongoengine_pddl_type = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if mongoengine_pddl_type:
            mongoengine_pddl_type.type_name = pddl_dto_type.get_type_name()
            mongoengine_pddl_type.save()
            return True

        else:
            return False

    def save(self, pddl_dto_type: PddlDtoType) -> bool:
        """ save or update a PddlDtoType
            if the PddlDtoType is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_type):
            return self._update(pddl_dto_type)

        else:
            return self._save(pddl_dto_type)

    def delete(self, pddl_dto_type: PddlDtoType) -> bool:
        """ delete a PddlDtoType
            if the PddlDtoType is not saved return False, else return True

        Args:
            pddl_dto_type (PddlDtoType): PddlDtoType to delete

        Returns:
            bool: succeed
        """

        mongoengine_pddl_type = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if mongoengine_pddl_type:
            mongoengine_pddl_type.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl types

        Returns:
            bool: succeed
        """

        pddl_dto_type_list = self.get_all()

        for ele in pddl_dto_type_list:
            self.delete(ele)

        return True
