
from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PDDL_DAO_Proposition
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_proposition as pddl_proposition_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_predicate as pddl_predicate_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_object as pddl_object_mongoengine_model

from pddl_dao.pddl_dto.pddl_dto_proposition import PDDL_DTO_Proposition

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import Mongoengine_PDDL_DAO_Object

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import Mongoengine_PDDL_DAO_Object
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate


class Mongoengine_PDDL_DAO_Proposition(PDDL_DAO_Proposition, Mongoengine_PDDL_DAO):

    def __init__(self, uri=None):

        PDDL_DAO_Proposition.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

        self._mongoengine_pddl_dao_object = Mongoengine_PDDL_DAO_Object(uri)
        self._mongoengine_pddl_dao_predicate = Mongoengine_PDDL_DAO_Predicate(
            uri)

    def _mongoengine_to_dto(self, pddl_proposition_mongoengine):

        pddl_object_list = []

        pddl_dto_predicate = self._mongoengine_pddl_dao_predicate._mongoengine_to_dto(
            pddl_proposition_mongoengine.pddl_predicate)

        # check if proposition is correct
        # if(len(pddl_proposition_mongoengine.pddl_objects) !=
        #   len(pddl_proposition_mongoengine.pddl_predicate.pddl_types)):
        #    return None

        for pddl_object, pddl_type in zip(pddl_proposition_mongoengine.pddl_objects,
                                          pddl_proposition_mongoengine.pddl_predicate.pddl_types):

            # check if proposition is correct
            # if(pddl_object.pddl_type.type_name != pddl_type.type_name):
            #    return None

            pddl_dto_object = self._mongoengine_pddl_dao_object._mongoengine_to_dto(
                pddl_object)

            # check if object exist
            # if(not pddl_dto_object):
            #    return None

            pddl_object_list.append(pddl_dto_object)

        pddl_dto_proposition = PDDL_DTO_Proposition(
            pddl_dto_predicate, pddl_object_list)

        pddl_dto_proposition.set_is_goal(
            pddl_proposition_mongoengine.is_goal)

        return pddl_dto_proposition

    def _check_pddl_dto_predicate_is_correct(self, pddl_dto_proposition):

       # check if proposition is correct
        if(len(pddl_dto_proposition.get_pddl_objects_list()) !=
           len(pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list())):
            return False

        for pddl_dto_object, pddl_dto_type in zip(pddl_dto_proposition.get_pddl_objects_list(),
                                                  pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list()):

            # check if proposition is correct
            if(pddl_dto_object.get_pddl_type().get_type_name() != pddl_dto_type.get_type_name()):
                return False

        return True

    def _dto_to_mongoengine(self, pddl_dto_proposition):

        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model()

        pddl_predicate_mongoengine = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_proposition.get_pddl_predicate())

        # check if predicate exist
        if(not pddl_predicate_mongoengine):
            return None

        pddl_proposition_mongoengine.pddl_predicate = pddl_predicate_mongoengine

        pddl_proposition_mongoengine.is_goal = pddl_dto_proposition.get_is_goal()

        for pddl_dto_object in pddl_dto_proposition.get_pddl_objects_list():

            pddl_object_mongoengine = self._mongoengine_pddl_dao_object._get_mongoengine(
                pddl_dto_object)

            # check if object exist
            if(not pddl_object_mongoengine):
                return None

            pddl_proposition_mongoengine.pddl_objects.append(
                pddl_object_mongoengine)

        return pddl_proposition_mongoengine

    def _exist_in_mongo(self, pddl_dto_proposition):

        if(self._get_mongoengine(pddl_dto_proposition)):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_proposition):

        pddl_dto_objects_list = pddl_dto_proposition.get_pddl_objects_list()

        pddl_predicate_mongoengine = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_proposition.get_pddl_predicate())

        # check if predicate exist
        if(not pddl_predicate_mongoengine):
            return None

        pddl_objects_list = []
        for pddl_object in pddl_dto_objects_list:
            pddl_object_mongoengine = self._mongoengine_pddl_dao_object._get_mongoengine(
                pddl_object)

            # check if object exist
            if(not pddl_object_mongoengine):
                return None
            pddl_objects_list.append(pddl_object_mongoengine)

        # getting proposition
        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model.objects(
            pddl_predicate=pddl_predicate_mongoengine, pddl_objects=pddl_objects_list)

        # check if proposition exist
        if(not pddl_proposition_mongoengine):
            return None

        return pddl_proposition_mongoengine[0]

    def get_by_predicate(self, predicate_name):

        pddl_predicate_mongoengine = pddl_predicate_mongoengine_model.objects(
            predicate_name=predicate_name)

        # check if predicate exist
        if(not pddl_predicate_mongoengine):
            return None
        pddl_predicate_mongoengine = pddl_predicate_mongoengine[0]

        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model.objects(
            pddl_predicate=pddl_predicate_mongoengine)

        pddl_dto_proposition_list = []

        for ele in pddl_proposition_mongoengine:
            pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def get_goals(self):

        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model.objects(
            is_goal=True)

        pddl_dto_proposition_list = []

        for ele in pddl_proposition_mongoengine:
            pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def get_all(self):

        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model.objects()

        pddl_dto_proposition_list = []

        for ele in pddl_proposition_mongoengine:
            pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def _save(self, pddl_dto_proposition):

        if(self._exist_in_mongo(pddl_dto_proposition)):
            return False

        if(not self._check_pddl_dto_predicate_is_correct(pddl_dto_proposition)):
            return False

       # propagating saving
        result = self._mongoengine_pddl_dao_predicate.save(
            pddl_dto_proposition.get_pddl_predicate())
        if(not result):
            return False
        for pddl_dto_object in pddl_dto_proposition.get_pddl_objects_list():
            result = self._mongoengine_pddl_dao_object.save(pddl_dto_object)
            if(not result):
                return False

        pddl_proposition_mongoengine = self._dto_to_mongoengine(
            pddl_dto_proposition)

        if(pddl_proposition_mongoengine):
            pddl_proposition_mongoengine.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_proposition):

        if(not self._check_pddl_dto_predicate_is_correct(pddl_dto_proposition)):
            return False

        pddl_proposition_mongoengine = self._get_mongoengine(
            pddl_dto_proposition)

        # check if proposition exists
        if(pddl_proposition_mongoengine):
            new_pddl_proposition_mongoengine = self._dto_to_mongoengine(
                pddl_dto_proposition)

            if(new_pddl_proposition_mongoengine):
                pddl_proposition_mongoengine.pddl_predicate = new_pddl_proposition_mongoengine.pddl_predicate
                pddl_proposition_mongoengine.pddl_objects = new_pddl_proposition_mongoengine.pddl_objects
                pddl_proposition_mongoengine.is_goal = new_pddl_proposition_mongoengine.is_goal
                pddl_proposition_mongoengine.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_proposition):

        if(self._exist_in_mongo(pddl_dto_proposition)):
            return self._update(pddl_dto_proposition)

        else:
            return self._save(pddl_dto_proposition)

    def delete(self, pddl_dto_proposition):

        pddl_proposition_mongoengine = self._get_mongoengine(
            pddl_dto_proposition)

        # check if proposition exists
        if(pddl_proposition_mongoengine):
            pddl_proposition_mongoengine.delete()
            return True

    def delete_all(self):

        pddl_dto_proposition_list = self.get_all()

        for ele in pddl_dto_proposition_list:
            self.delete(ele)

        return True
