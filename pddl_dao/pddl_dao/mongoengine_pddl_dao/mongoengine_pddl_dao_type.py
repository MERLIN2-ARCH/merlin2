
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_type import PDDL_DAO_Type
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_type as pddl_type_mongoengine_model

from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type


class Mongoengine_PDDL_DAO_Type(PDDL_DAO_Type, Mongoengine_PDDL_DAO):

    def __init__(self, uri: str = None):

        PDDL_DAO_Type.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

    def _mongoengine_to_dto(self, pddl_type_mongoengine: pddl_type_mongoengine_model) -> PDDL_DTO_Type:
        pddl_dto_type = PDDL_DTO_Type(pddl_type_mongoengine.type_name)
        return pddl_dto_type

    def _dto_to_mongoengine(self, pddl_dto_type: PDDL_DTO_Type) -> pddl_type_mongoengine_model:
        pddl_type_mongoengine = pddl_type_mongoengine_model()
        pddl_type_mongoengine.type_name = pddl_dto_type.get_type_name()
        return pddl_type_mongoengine

    def _exist_in_mongo(self, pddl_dto_type: PDDL_DTO_Type) -> bool:

        if(self._get_mongoengine(pddl_dto_type)):
            return True
        return False

    def _get_mongoengine(self, pddl_dao_type: PDDL_DTO_Type) -> pddl_type_mongoengine_model:
        pddl_type_mongoengine = pddl_type_mongoengine_model.objects(
            type_name=pddl_dao_type.get_type_name())

        if(not pddl_type_mongoengine):
            return None

        return pddl_type_mongoengine[0]

    def get(self, type_name: str) -> PDDL_DTO_Type:

        pddl_type_mongoengine = pddl_type_mongoengine_model.objects(
            type_name=type_name)

        if(pddl_type_mongoengine):
            pddl_type_mongoengine = pddl_type_mongoengine[0]
            return self._mongoengine_to_dto(pddl_type_mongoengine)

        else:
            return None

    def get_all(self) -> List[PDDL_DTO_Type]:

        pddl_type_mongoengine = pddl_type_mongoengine_model.objects
        pddl_dto_type_list = []

        for ele in pddl_type_mongoengine:
            pddl_dto_type = self._mongoengine_to_dto(ele)
            pddl_dto_type_list.append(pddl_dto_type)

        return pddl_dto_type_list

    def _save(self, pddl_dto_type: PDDL_DTO_Type) -> bool:

        if(self._exist_in_mongo(pddl_dto_type)):
            return False

        pddl_type_mongoengine = self._dto_to_mongoengine(pddl_dto_type)
        pddl_type_mongoengine.save()
        return True

    def _update(self, pddl_dto_type: PDDL_DTO_Type) -> bool:

        pddl_type_mongoengine = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if(pddl_type_mongoengine):
            pddl_type_mongoengine.type_name = pddl_dto_type.get_type_name()
            pddl_type_mongoengine.save()
            return True

        else:
            return False

    def save(self, pddl_dto_type: PDDL_DTO_Type) -> bool:

        if(self._exist_in_mongo(pddl_dto_type)):
            return self._update(pddl_dto_type)

        else:
            return self._save(pddl_dto_type)

    def delete(self, pddl_dto_type: PDDL_DTO_Type) -> bool:

        pddl_type_mongoengine = self._get_mongoengine(pddl_dto_type)

        # check if type exists
        if(pddl_type_mongoengine):
            pddl_type_mongoengine.delete()
            return True

        return False

    def delete_all(self) -> bool:

        pddl_dto_type_list = self.get_all()

        for ele in pddl_dto_type_list:
            self.delete(ele)

        return True
