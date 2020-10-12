
from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_predicate import PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import pddl_predicate as mongoengine_pddl_predicate_model

from pddl_dao.pddl_dto.pddl_dto_predicate import PDDL_DTO_Predicate
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import Mongoengine_PDDL_DAO_Type


class Mongoengine_PDDL_DAO_Predicate(PDDL_DAO_Predicate, Mongoengine_PDDL_DAO):

    def __init__(self, uri: str = None):

        PDDL_DAO_Predicate.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

        self._mongoengine_pddl_dao_type = Mongoengine_PDDL_DAO_Type(uri)

    def _mongoengine_to_dto(self, mongoengine_pddl_predicate: mongoengine_pddl_predicate_model) -> PDDL_DTO_Predicate:

        pddl_dto_type_list = []

        for pddl_type in mongoengine_pddl_predicate.pddl_types:
            pddl_dto_type = PDDL_DTO_Type(pddl_type.type_name)
            pddl_dto_type_list.append(pddl_dto_type)

        pddl_dto_predicate = PDDL_DTO_Predicate(
            mongoengine_pddl_predicate.predicate_name, pddl_dto_type_list)

        return pddl_dto_predicate

    def _dto_to_mongoengine(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> mongoengine_pddl_predicate_model:

        mongoengine_pddl_predicate = mongoengine_pddl_predicate_model()

        mongoengine_pddl_predicate.predicate_name = pddl_dto_predicate.get_predicate_name()

        for pddl_dto_type in pddl_dto_predicate.get_pddl_types_list():

            mongoengine_pddl_type = self._mongoengine_pddl_dao_type._get_mongoengine(
                pddl_dto_type)

            # check if type exist
            if(not mongoengine_pddl_type):
                return None

            mongoengine_pddl_predicate.pddl_types.append(mongoengine_pddl_type)

        return mongoengine_pddl_predicate

    def _exist_in_mongo(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> bool:

        if(self._get_mongoengine(pddl_dto_predicate)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> mongoengine_pddl_predicate_model:

        mongoengine_pddl_predicate = mongoengine_pddl_predicate_model.objects(
            predicate_name=pddl_dto_predicate.get_predicate_name())

        if(not mongoengine_pddl_predicate):
            return None

        return mongoengine_pddl_predicate[0]

    def get(self, predicate_name: str) -> PDDL_DTO_Predicate:

        mongoengine_pddl_predicate = mongoengine_pddl_predicate_model.objects(
            predicate_name=predicate_name)

        # check if predicate exist
        if(mongoengine_pddl_predicate):
            mongoengine_pddl_predicate = mongoengine_pddl_predicate[0]
            pddl_dto_predicate = self._mongoengine_to_dto(
                mongoengine_pddl_predicate)
            return pddl_dto_predicate

        else:
            return None

    def get_all(self) -> List[PDDL_DTO_Predicate]:

        mongoengine_pddl_predicate = mongoengine_pddl_predicate_model.objects
        pddl_dto_predicate_list = []

        for ele in mongoengine_pddl_predicate:
            pddl_dto_predicate = self._mongoengine_to_dto(ele)
            pddl_dto_predicate_list.append(pddl_dto_predicate)

        return pddl_dto_predicate_list

    def _save(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> bool:

        if(self._exist_in_mongo(pddl_dto_predicate)):
            return False

        # propagating saving
        for pddl_dto_type in pddl_dto_predicate.get_pddl_types_list():
            result = self._mongoengine_pddl_dao_type.save(pddl_dto_type)
            if(not result):
                return False

        mongoengine_pddl_predicate = self._dto_to_mongoengine(
            pddl_dto_predicate)

        if(mongoengine_pddl_predicate):
            mongoengine_pddl_predicate.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> bool:

        mongoengine_pddl_predicate = self._get_mongoengine(pddl_dto_predicate)

        # check if predicate exists
        if(mongoengine_pddl_predicate):
            new_pddl_predicate_mongoengine = self._dto_to_mongoengine(
                pddl_dto_predicate)

            if(new_pddl_predicate_mongoengine):
                mongoengine_pddl_predicate.predicate_name = new_pddl_predicate_mongoengine.predicate_name
                mongoengine_pddl_predicate.pddl_types = new_pddl_predicate_mongoengine.pddl_types
                mongoengine_pddl_predicate.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> bool:

        if(self._exist_in_mongo(pddl_dto_predicate)):
            return self._update(pddl_dto_predicate)

        else:
            return self._save(pddl_dto_predicate)

    def delete(self, pddl_dto_predicate: PDDL_DTO_Predicate) -> bool:

        mongoengine_pddl_predicate = self._get_mongoengine(pddl_dto_predicate)

        # check if predicate exists
        if(mongoengine_pddl_predicate):
            mongoengine_pddl_predicate.delete()
            return True

    def delete_all(self) -> bool:

        pddl_dto_predicate_list = self.get_all()

        for ele in pddl_dto_predicate_list:
            self.delete(ele)

        return True
