
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_object import PDDL_DAO_Object
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import pddl_object as mongoengine_pddl_object_model

from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import Mongoengine_PDDL_DAO_Type


class Mongoengine_PDDL_DAO_Object(PDDL_DAO_Object, Mongoengine_PDDL_DAO):

    def __init__(self, uri: str = None):

        PDDL_DAO_Object.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

        self._mongoengine_pddl_dao_type = Mongoengine_PDDL_DAO_Type(uri)

    def _mongoengine_to_dto(self, mongoengine_pddl_object: mongoengine_pddl_object_model) -> PDDL_DTO_Object:

        pddl_dto_type = PDDL_DTO_Type(
            mongoengine_pddl_object.pddl_type.type_name)

        pddl_dto_object = PDDL_DTO_Object(pddl_dto_type,
                                          mongoengine_pddl_object.object_name)

        return pddl_dto_object

    def _dto_to_mongoengine(self, pddl_dto_object: PDDL_DTO_Object) -> mongoengine_pddl_object_model:

        mongoengine_pddl_type = self._mongoengine_pddl_dao_type._get_mongoengine(
            pddl_dto_object.get_pddl_type())

        if(not pddl_dto_object):
            return None

        mongoengine_pddl_object = mongoengine_pddl_object_model()

        mongoengine_pddl_object.object_name = pddl_dto_object.get_object_name()

        mongoengine_pddl_object.pddl_type = mongoengine_pddl_type

        return mongoengine_pddl_object

    def _exist_in_mongo(self, pddl_dto_object: PDDL_DTO_Object) -> bool:

        if(self._get_mongoengine(pddl_dto_object)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_object: PDDL_DTO_Object) -> bool:
        mongoengine_pddl_object = mongoengine_pddl_object_model.objects(
            object_name=pddl_dto_object.get_object_name())

        if(not mongoengine_pddl_object):
            return None

        return mongoengine_pddl_object[0]

    def get(self, object_name: str) -> PDDL_DTO_Object:

        mongoengine_pddl_object = mongoengine_pddl_object_model.objects(
            object_name=object_name)

        # check if object exists
        if(mongoengine_pddl_object):
            mongoengine_pddl_object = mongoengine_pddl_object[0]
            pddl_dto_object = self._mongoengine_to_dto(
                mongoengine_pddl_object)
            return pddl_dto_object

        else:
            return None

    def get_all(self) -> List[PDDL_DTO_Object]:

        mongoengine_pddl_object = mongoengine_pddl_object_model.objects
        pddl_dto_object_list = []

        for ele in mongoengine_pddl_object:
            pddl_dto_object = self._mongoengine_to_dto(ele)
            pddl_dto_object_list.append(pddl_dto_object)

        return pddl_dto_object_list

    def _save(self, pddl_dto_object: PDDL_DTO_Object) -> bool:

        if(self._exist_in_mongo(pddl_dto_object)):
            return False

        # propagating saving
        result = self._mongoengine_pddl_dao_type.save(
            pddl_dto_object.get_pddl_type())

        if(not result):
            return False

        mongoengine_pddl_object = self._dto_to_mongoengine(
            pddl_dto_object)

        if(mongoengine_pddl_object):
            mongoengine_pddl_object.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_object: PDDL_DTO_Object) -> bool:

        mongoengine_pddl_object = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if(mongoengine_pddl_object):
            new_pddl_object_mongoengine = self._dto_to_mongoengine(
                pddl_dto_object)

            if(new_pddl_object_mongoengine):
                mongoengine_pddl_object.object_name = new_pddl_object_mongoengine.object_name
                mongoengine_pddl_object.pddl_type = new_pddl_object_mongoengine.pddl_type
                mongoengine_pddl_object.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_object: PDDL_DTO_Object) -> bool:

        if(self._exist_in_mongo(pddl_dto_object)):
            return self._update(pddl_dto_object)

        else:
            return self._save(pddl_dto_object)

    def delete(self, pddl_dto_object: PDDL_DTO_Object) -> bool:

        mongoengine_pddl_object = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if(mongoengine_pddl_object):
            mongoengine_pddl_object.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_object_list = self.get_all()

        for ele in pddl_dto_object_list:
            self.delete(ele)

        return True
