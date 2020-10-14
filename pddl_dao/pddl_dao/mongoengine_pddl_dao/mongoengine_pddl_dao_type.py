
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_type import PDDL_DAO_Type
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import pddl_type as mongoengine_pddl_type_model

from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class Mongoengine_PDDL_DAO_Type(PDDL_DAO_Type, Mongoengine_PDDL_DAO):

    def __init__(self, uri: str = None):

        PDDL_DAO_Type.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

    def _mongoengine_to_dto(self, mongoengine_pddl_type: mongoengine_pddl_type_model) -> PddlDtoType:

        pddl_dto_type = PddlDtoType(mongoengine_pddl_type.type_name)
        return pddl_dto_type

    def _dto_to_mongoengine(self, pddl_dto_type: PddlDtoType) -> mongoengine_pddl_type_model:

        mongoengine_pddl_type = mongoengine_pddl_type_model()
        mongoengine_pddl_type.type_name = pddl_dto_type.get_type_name()
        return mongoengine_pddl_type

    def _exist_in_mongo(self, pddl_dto_type: PddlDtoType) -> bool:

        if(self._get_mongoengine(pddl_dto_type)):
            return True
        return False

    def _get_mongoengine(self, pddl_dao_type: PddlDtoType) -> mongoengine_pddl_type_model:
        mongoengine_pddl_type = mongoengine_pddl_type_model.objects(
            type_name=pddl_dao_type.get_type_name())

        if(not mongoengine_pddl_type):
            return None

        return mongoengine_pddl_type[0]

    def get(self, type_name: str) -> PddlDtoType:

        mongoengine_pddl_type = mongoengine_pddl_type_model.objects(
            type_name=type_name)

        if(mongoengine_pddl_type):
            mongoengine_pddl_type = mongoengine_pddl_type[0]
            return self._mongoengine_to_dto(mongoengine_pddl_type)

        else:
            return None

    def get_all(self) -> List[PddlDtoType]:

        mongoengine_pddl_type = mongoengine_pddl_type_model.objects
        pddl_dto_type_list = []

        for ele in mongoengine_pddl_type:
            pddl_dto_type = self._mongoengine_to_dto(ele)
            pddl_dto_type_list.append(pddl_dto_type)

        return pddl_dto_type_list

    def _save(self, pddl_dto_type: PddlDtoType) -> bool:

        if(self._exist_in_mongo(pddl_dto_type)):
            return False

        mongoengine_pddl_type = self._dto_to_mongoengine(pddl_dto_type)
        mongoengine_pddl_type.save()
        return True

    def _update(self, pddl_dto_type: PddlDtoType) -> bool:

        mongoengine_pddl_type = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if(mongoengine_pddl_type):
            mongoengine_pddl_type.type_name = pddl_dto_type.get_type_name()
            mongoengine_pddl_type.save()
            return True

        else:
            return False

    def save(self, pddl_dto_type: PddlDtoType) -> bool:

        if(self._exist_in_mongo(pddl_dto_type)):
            return self._update(pddl_dto_type)

        else:
            return self._save(pddl_dto_type)

    def delete(self, pddl_dto_type: PddlDtoType) -> bool:

        mongoengine_pddl_type = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if(mongoengine_pddl_type):
            mongoengine_pddl_type.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_type_list = self.get_all()

        for ele in pddl_dto_type_list:
            self.delete(ele)

        return True
