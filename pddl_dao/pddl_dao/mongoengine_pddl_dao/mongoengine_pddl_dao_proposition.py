
from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PDDL_DAO_Proposition
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import Mongoengine_PDDL_DAO

from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_proposition as pddl_proposition_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_predicate as pddl_predicate_mongoengine_model
from pddl_dao.mongoengine_pddl_dao.pddl_mongoengine_models import pddl_object as pddl_object_mongoengine_model

from pddl_dao.pddl_dto.pddl_dto_proposition import PDDL_DTO_Proposition

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import Mongoengine_PDDL_DAO_Predicate
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import Mongoengine_PDDL_DAO_Object


class Mongoengine_PDDL_DAO_Proposition(PDDL_DAO_Proposition, Mongoengine_PDDL_DAO):

    def __init__(self, uri=None):

        PDDL_DAO_Proposition.__init__(self)
        Mongoengine_PDDL_DAO.__init__(self, uri)

    def _mongoengine_to_dto(self, pddl_proposition_mongoengine):

        pddl_predicate_dao = Mongoengine_PDDL_DAO_Predicate(self.get_uri())
        pddl_object_dao = Mongoengine_PDDL_DAO_Object(self.get_uri())
        pddl_object_list = []

        pddl_dto_predicate = pddl_predicate_dao.get(
            pddl_proposition_mongoengine.pddl_predicate.predicate_name)

        # check if predicate exist
        # if(not pddl_dto_predicate):
        #    return None

        # check if proposition is correct
        # if(len(pddl_proposition_mongoengine.pddl_objects) !=
        #   len(pddl_proposition_mongoengine.pddl_predicate.pddl_types)):
        #    return None

        for pddl_object, pddl_type in zip(pddl_proposition_mongoengine.pddl_objects,
                                          pddl_proposition_mongoengine.pddl_predicate.pddl_types):

            # check if proposition is correct
            # if(pddl_object.pddl_type.type_name != pddl_type.type_name):
            #    return None

            pddl_dto_object = pddl_object_dao.get(pddl_object.object_name)

            # check if object exist
            # if(not pddl_dto_object):
            #    return None

            pddl_object_list.append(pddl_dto_object)

        pddl_dto_proposition = PDDL_DTO_Proposition(
            pddl_dto_predicate, pddl_object_list)

        pddl_dto_proposition.set_is_goal(
            pddl_proposition_mongoengine.is_goal)

        return pddl_dto_proposition

    def _dto_to_mongoengine(self, pddl_dto_proposition):

       # check if proposition is correct
        if(len(pddl_dto_proposition.get_pddl_objects_list()) !=
           len(pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list())):
            return None

        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model()

        pddl_predicate_mongoengine = pddl_predicate_mongoengine_model.objects(
            predicate_name=pddl_dto_proposition.get_pddl_predicate().get_predicate_name())

        # check if predicate exist
        if(not pddl_predicate_mongoengine):
            return None

        pddl_proposition_mongoengine.pddl_predicate = pddl_predicate_mongoengine[0]

        pddl_proposition_mongoengine.is_goal = pddl_dto_proposition.get_is_goal()

        for pddl_dto_object, pddl_dto_type in zip(pddl_dto_proposition.get_pddl_objects_list(),
                                                  pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list()):

            # check if proposition is correct
            if(pddl_dto_object.get_pddl_type().get_type_name() != pddl_dto_type.get_type_name()):
                return None

            pddl_object_mongoengine = pddl_object_mongoengine_model.objects(
                object_name=pddl_dto_object.get_object_name())

            # check if object exist
            if(not pddl_object_mongoengine):
                return None

            pddl_proposition_mongoengine.pddl_objects.append(
                pddl_object_mongoengine[0])

        return pddl_proposition_mongoengine

    def __get_pddl_proposition_mongoengine(self, pddl_dto_proposition):

        predicate_name = pddl_dto_proposition.get_pddl_predicate().get_predicate_name()
        pddl_dto_objects_list = pddl_dto_proposition.get_pddl_objects_list()

        pddl_predicate_mongoengine = pddl_predicate_mongoengine_model.objects(
            predicate_name=predicate_name)

        # check if predicate exist
        if(not pddl_predicate_mongoengine):
            return None
        pddl_predicate_mongoengine = pddl_predicate_mongoengine[0]

        pddl_objects_list = []
        for pddl_object in pddl_dto_objects_list:
            pddl_object_mongoengine = pddl_object_mongoengine_model.objects(
                object_name=pddl_object.get_object_name())

            # check if object exist
            if(not pddl_object_mongoengine):
                return None
            pddl_objects_list.append(pddl_object_mongoengine[0])

        # getting proposition
        pddl_proposition_mongoengine = pddl_proposition_mongoengine_model.objects(
            pddl_predicate=pddl_predicate_mongoengine, pddl_objects=pddl_objects_list)

        # check if proposition exist
        if(not pddl_proposition_mongoengine):
            return None

        return pddl_proposition_mongoengine[0]

    def _exist_in_mongo(self, pddl_dto_proposition):

        if(self.__get_pddl_proposition_mongoengine(pddl_dto_proposition)):
            return True
        else:
            return False

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

    def save(self, pddl_dto_proposition):

        if(self._exist_in_mongo(pddl_dto_proposition)):
            return False

        pddl_proposition_mongoengine = self._dto_to_mongoengine(
            pddl_dto_proposition)

        if(pddl_proposition_mongoengine):
            pddl_proposition_mongoengine.save()
            return True

        else:
            return False

    def update(self, pddl_dto_proposition):

        pddl_proposition_mongoengine = self.__get_pddl_proposition_mongoengine(
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

    def save_update(self, pddl_dto_proposition):

        if(self._exist_in_mongo(pddl_dto_proposition)):
            return self.update(pddl_dto_proposition)

        else:
            return self.save(pddl_dto_proposition)

    def delete(self, pddl_dto_proposition):

        pddl_proposition_mongoengine = self.__get_pddl_proposition_mongoengine(
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
