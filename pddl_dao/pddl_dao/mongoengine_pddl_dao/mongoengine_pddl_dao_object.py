
from pddl_dao.pddl_dao_interface.pddl_dao_object import PDDL_DAO_Object
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_object as pddl_object_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_type as pddl_type_mongoengine_model

from pddl_dao.pddl_dto.pddl_dto_object import PDDL_DTO_Object
from pddl_dao.pddl_dto.pddl_dto_type import PDDL_DTO_Type

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_type import Mongoengine_PDDL_DAO_Type


class Mongoengine_PDDL_DAO_Object(PDDL_DAO_Object, Mongoengine_PDDL_DAO):

    def __init__(self, uri=None):

        PDDL_DAO_Object.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

        self._mongoengine_pddl_dao_type = Mongoengine_PDDL_DAO_Type(uri)

    def _mongoengine_to_dto(self, pddl_object_mongoengine):

        pddl_dto_type = PDDL_DTO_Type(
            pddl_object_mongoengine.pddl_type.type_name)

        pddl_dto_object = PDDL_DTO_Object(pddl_dto_type,
                                          pddl_object_mongoengine.object_name)

        return pddl_dto_object

    def _dto_to_mongoengine(self, pddl_dto_object):

        pddl_type_mongoengine = self._mongoengine_pddl_dao_type._get_mongoengine(
            pddl_dto_object.get_pddl_type())

        if(not pddl_dto_object):
            return None

        pddl_object_mongoengine = pddl_object_mongoengine_model()

        pddl_object_mongoengine.object_name = pddl_dto_object.get_object_name()

        pddl_object_mongoengine.pddl_type = pddl_type_mongoengine

        return pddl_object_mongoengine

    def _exist_in_mongo(self, pddl_dto_object):

        if(self._get_mongoengine(pddl_dto_object)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_object):
        pddl_object_mongoengine = pddl_object_mongoengine_model.objects(
            object_name=pddl_dto_object.get_object_name())

        if(not pddl_object_mongoengine):
            return None

        return pddl_object_mongoengine[0]

    def get(self, object_name):

        pddl_object_mongoengine = pddl_object_mongoengine_model.objects(
            object_name=object_name)

        # check if object exists
        if(pddl_object_mongoengine):
            pddl_object_mongoengine = pddl_object_mongoengine[0]
            pddl_dto_object = self._mongoengine_to_dto(
                pddl_object_mongoengine)
            return pddl_dto_object

        else:
            return None

    def get_all(self):

        pddl_object_mongoengine = pddl_object_mongoengine_model.objects
        pddl_dto_object_list = []

        for ele in pddl_object_mongoengine:
            pddl_dto_object = self._mongoengine_to_dto(ele)
            pddl_dto_object_list.append(pddl_dto_object)

        return pddl_dto_object_list

    def _save(self, pddl_dto_object):

        if(self._exist_in_mongo(pddl_dto_object)):
            return False

        # propagating saving
        result = self._mongoengine_pddl_dao_type.save(
            pddl_dto_object.get_pddl_type())

        if(not result):
            return False

        pddl_object_mongoengine = self._dto_to_mongoengine(
            pddl_dto_object)

        if(pddl_object_mongoengine):
            pddl_object_mongoengine.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_object):

        pddl_object_mongoengine = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if(pddl_object_mongoengine):
            new_pddl_object_mongoengine = self._dto_to_mongoengine(
                pddl_dto_object)

            if(new_pddl_object_mongoengine):
                pddl_object_mongoengine.object_name = new_pddl_object_mongoengine.object_name
                pddl_object_mongoengine.pddl_type = new_pddl_object_mongoengine.pddl_type
                pddl_object_mongoengine.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_object):

        if(self._exist_in_mongo(pddl_dto_object)):
            return self._update(pddl_dto_object)

        else:
            return self._save(pddl_dto_object)

    def delete(self, pddl_dto_object):

        pddl_object_mongoengine = self._get_mongoengine(pddl_dto_object)

        # check if object exists
        if(pddl_object_mongoengine):
            pddl_object_mongoengine.delete()
            return True

        return False

    def delete_all(self):
        pddl_dto_object_list = self.get_all()

        for ele in pddl_dto_object_list:
            self.delete(ele)

        return True
