
""" Mongoengine Pddl Dao Object """

from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_object import PddlDaoObject
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import (
    PddlObjectModel as mongoengine_pddl_object_model
)

from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import MongoenginePddlDaoType


class MongoenginePddlDaoObject(PddlDaoObject, MongoenginePddlDao):
    """ Mongoengine Pddl Dao Object Class """

    def __init__(self, uri: str = None):

        PddlDaoObject.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._mongoengine_pddl_dao_type = MongoenginePddlDaoType(uri)

    def _mongoengine_to_dto(self, mongoengine_pddl_object: mongoengine_pddl_object_model) -> PddlDtoObject:
        """ convert a Mongoengine pddl object document into a PddlDtoObject

        Args:
            mongoengine_pddl_object (mongoengine_pddl_object_model): Mongoengine pddl object document

        Returns:
            PddlDtoObject: PddlDtoObject
        """

        pddl_dto_type = PddlDtoType(
            mongoengine_pddl_object.PddlTypeModel.type_name)

        pddl_dto_object = PddlDtoObject(pddl_dto_type,
                                        mongoengine_pddl_object.object_name)

        return pddl_dto_object

    def _dto_to_mongoengine(self, pddl_dto_object: PddlDtoObject) -> mongoengine_pddl_object_model:
        """ convert a PddlDtoObject into a Mongoengine pddl object document

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject

        Returns:
            Document: Mongoengine pddl object document
        """

        mongoengine_pddl_type = self._mongoengine_pddl_dao_type._get_mongoengine(
            pddl_dto_object.get_pddl_type())

        if not pddl_dto_object:
            return None

        mongoengine_pddl_object = mongoengine_pddl_object_model()

        mongoengine_pddl_object.object_name = pddl_dto_object.get_object_name()

        mongoengine_pddl_object.PddlTypeModel = mongoengine_pddl_type

        return mongoengine_pddl_object

    def _exist_in_mongo(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ check if PddlDtoObject exists

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject

        Returns:
            bool: PddlDtoObject exists?
        """

        if self._get_mongoengine(pddl_dto_object):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ get the Mongoengine pddl object document corresponding to a give PddlDtoObject

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject

        Returns:
            Document: Mongoengine pddl object document
        """

        mongoengine_pddl_object = mongoengine_pddl_object_model.objects(
            object_name=pddl_dto_object.get_object_name())

        if not mongoengine_pddl_object:
            return None

        return mongoengine_pddl_object[0]

    def get(self, object_name: str) -> PddlDtoObject:
        """ get a PddlDtoObject with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlDtoObject: PddlDtoObject of the pddl object name
        """

        mongoengine_pddl_object = mongoengine_pddl_object_model.objects(
            object_name=object_name)

        # check if object exists
        if mongoengine_pddl_object:
            mongoengine_pddl_object = mongoengine_pddl_object[0]
            pddl_dto_object = self._mongoengine_to_dto(
                mongoengine_pddl_object)
            return pddl_dto_object

        else:
            return None

    def get_all(self) -> List[PddlDtoObject]:
        """ get all PddlDtoObject

        Returns:
            List[PddlDtoObject]: list of all PddlDtoObject
        """

        mongoengine_pddl_object = mongoengine_pddl_object_model.objects
        pddl_dto_object_list = []

        for ele in mongoengine_pddl_object:
            pddl_dto_object = self._mongoengine_to_dto(ele)
            pddl_dto_object_list.append(pddl_dto_object)

        return pddl_dto_object_list

    def _save(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ save a PddlDtoObject
            if the PddlDtoObject is already saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_object):
            return False

        # propagating saving
        result = self._mongoengine_pddl_dao_type.save(
            pddl_dto_object.get_pddl_type())

        if not result:
            return False

        mongoengine_pddl_object = self._dto_to_mongoengine(
            pddl_dto_object)

        if mongoengine_pddl_object:
            mongoengine_pddl_object.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ update a PddlDtoObject
            if the PddlDtoObject is not saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to update

        Returns:
            bool: succeed
        """

        mongoengine_pddl_object = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if mongoengine_pddl_object:
            new_pddl_object_mongoengine = self._dto_to_mongoengine(
                pddl_dto_object)

            if new_pddl_object_mongoengine:
                mongoengine_pddl_object.object_name = new_pddl_object_mongoengine.object_name
                mongoengine_pddl_object.PddlTypeModel = new_pddl_object_mongoengine.PddlTypeModel
                mongoengine_pddl_object.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ save or update a PddlDtoObject
            if the PddlDtoObject is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_object):
            return self._update(pddl_dto_object)

        else:
            return self._save(pddl_dto_object)

    def delete(self, pddl_dto_object: PddlDtoObject) -> bool:
        """ delete a PddlDtoObject
            if the PddlDtoObject is not saved return False, else return True

        Args:
            pddl_dto_object (PddlDtoObject): PddlDtoObject to delete

        Returns:
            bool: succeed
        """

        mongoengine_pddl_object = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if mongoengine_pddl_object:
            mongoengine_pddl_object.delete()
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
