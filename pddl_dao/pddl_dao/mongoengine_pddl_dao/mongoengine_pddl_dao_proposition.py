
""" Mongoengine Pddl Dao Proposition """

from typing import List

from pddl_dao.pddl_dao_interface.pddl_dao_proposition import PddlDaoProposition
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao import MongoenginePddlDao

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import(
    PddlPropositionModel as mongoengine_pddl_proposition_model
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_models import (
    PddlPredicateModel as mongoengine_pddl_predicate_model
)

from pddl_dao.pddl_dto.pddl_dto_proposition import PddlDtoProposition

from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_predicate import (
    MongoenginePddlDaoPredicate
)
from pddl_dao.mongoengine_pddl_dao.mongoengine_pddl_dao_object import (
    MongoenginePddlDaoObject
)


class MongoenginePddlDaoProposition(PddlDaoProposition, MongoenginePddlDao):
    """ Mongoengine Pddl Dao Proposition Class """

    def __init__(self, uri: str = None):

        PddlDaoProposition.__init__(self)
        MongoenginePddlDao.__init__(self, uri)

        self._mongoengine_pddl_dao_object = MongoenginePddlDaoObject(uri)
        self._mongoengine_pddl_dao_predicate = MongoenginePddlDaoPredicate(
            uri)

    def _check_mongoengine_pddl_proposition_is_correct(self, mongoengine_pddl_proposition: mongoengine_pddl_proposition_model) -> bool:
        """ check if the types of the objects of a pddl proposition model are
            the same as the types of its predicate

        Args:
            mongoengine_pddl_proposition (mongoengine_pddl_proposition_model): Mongoengine pddl proposition document

        Returns:
            bool: poposition is correct?
        """

        # check if proposition is correct
        if len(mongoengine_pddl_proposition.pddl_objects) != len(mongoengine_pddl_proposition.PddlPredicateModel.pddl_types):
            return False

        for pddl_object_model, pddl_type_model in zip(mongoengine_pddl_proposition.pddl_objects,
                                                      mongoengine_pddl_proposition.PddlPredicateModel.pddl_types):

            # check if proposition is correct
            if pddl_object_model.PddlTypeModel.type_name != pddl_type_model.type_name:
                return False

        return True

    def _mongoengine_to_dto(self, mongoengine_pddl_proposition: mongoengine_pddl_proposition_model) -> PddlDtoProposition:
        """ convert a Mongoengine pddl type document into a PddlDtoProposition

        Args:
            mongoengine_pddl_proposition (mongoengine_pddl_proposition_model): Mongoengine pddl proposition document

        Returns:
            PddlDto: PddlDtoProposition
        """

        pddl_object_list = []

        pddl_dto_predicate = self._mongoengine_pddl_dao_predicate._mongoengine_to_dto(
            mongoengine_pddl_proposition.PddlPredicateModel)

        for pddl_object_model in mongoengine_pddl_proposition.pddl_objects:

            pddl_dto_object = self._mongoengine_pddl_dao_object._mongoengine_to_dto(
                pddl_object_model)

            # check if object exist
            if not pddl_dto_object:
                return None

            pddl_object_list.append(pddl_dto_object)

        pddl_dto_proposition = PddlDtoProposition(
            pddl_dto_predicate, pddl_object_list)

        pddl_dto_proposition.set_is_goal(
            mongoengine_pddl_proposition.is_goal)

        return pddl_dto_proposition

    def _check_pddl_dto_proposition_is_correct(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ check if the types of the objects of a pddl proposition dto are
            the same as the types of its predicate

        Args:
            pddl_dto_proposition (PddlDtoProposition): pddl proposition dto

        Returns:
            bool: poposition is correct?
        """

       # check if proposition is correct
        if len(pddl_dto_proposition.get_pddl_objects_list()) != len(pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list()):
            return False

        for pddl_dto_object, pddl_dto_type in zip(pddl_dto_proposition.get_pddl_objects_list(),
                                                  pddl_dto_proposition.get_pddl_predicate().get_pddl_types_list()):

            # check if proposition is correct
            if pddl_dto_object.get_pddl_type().get_type_name() != pddl_dto_type.get_type_name():
                return False

        return True

    def _dto_to_mongoengine(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ convert a PddlDtoProposition into a Mongoengine pddl propostion document

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition

        Returns:
            Document: Mongoengine pddl propostion document
        """

        mongoengine_pddl_proposition = mongoengine_pddl_proposition_model()

        mongoengine_pddl_predicate = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_proposition.get_pddl_predicate())

        # check if predicate exist
        if not mongoengine_pddl_predicate:
            return None

        mongoengine_pddl_proposition.PddlPredicateModel = mongoengine_pddl_predicate

        mongoengine_pddl_proposition.is_goal = pddl_dto_proposition.get_is_goal()

        for pddl_dto_object in pddl_dto_proposition.get_pddl_objects_list():

            mongoengine_pddl_object = self._mongoengine_pddl_dao_object._get_mongoengine(
                pddl_dto_object)

            # check if object exist
            if not mongoengine_pddl_object:
                return None

            mongoengine_pddl_proposition.pddl_objects.append(
                mongoengine_pddl_object)

        return mongoengine_pddl_proposition

    def _exist_in_mongo(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ check if PddlDtoProposition exists

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition

        Returns:
            bool: PddlDtoProposition exists?
        """

        if self._get_mongoengine(pddl_dto_proposition):
            return True
        return False

    def _get_mongoengine(self, pddl_dto_proposition: PddlDtoProposition) -> mongoengine_pddl_proposition_model:
        """ get the Mongoengine pddl proposition document corresponding to a give PddlDtoProposition

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition

        Returns:
            Document: Mongoengine pddl proposition document
        """

        pddl_dto_objects_list = pddl_dto_proposition.get_pddl_objects_list()

        mongoengine_pddl_predicate = self._mongoengine_pddl_dao_predicate._get_mongoengine(
            pddl_dto_proposition.get_pddl_predicate())

        # check if predicate exist
        if not mongoengine_pddl_predicate:
            return None

        pddl_objects_list = []
        for PddlObjectModel in pddl_dto_objects_list:
            mongoengine_pddl_object = self._mongoengine_pddl_dao_object._get_mongoengine(
                PddlObjectModel)

            # check if object exist
            if not mongoengine_pddl_object:
                return None
            pddl_objects_list.append(mongoengine_pddl_object)

        # getting proposition
        mongoengine_pddl_proposition = mongoengine_pddl_proposition_model.objects(
            PddlPredicateModel=mongoengine_pddl_predicate, pddl_objects=pddl_objects_list)

        # check if proposition exist
        if not mongoengine_pddl_proposition:
            return None

        return mongoengine_pddl_proposition[0]

    def get_by_predicate(self, predicate_name: str) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition with a given pddl predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

        mongoengine_pddl_predicate = mongoengine_pddl_predicate_model.objects(
            predicate_name=predicate_name)

        # check if predicate exist
        if not mongoengine_pddl_predicate:
            return None
        mongoengine_pddl_predicate = mongoengine_pddl_predicate[0]

        mongoengine_pddl_proposition = mongoengine_pddl_proposition_model.objects(
            PddlPredicateModel=mongoengine_pddl_predicate)

        pddl_dto_proposition_list = []

        for ele in mongoengine_pddl_proposition:
            if self._check_mongoengine_pddl_proposition_is_correct(ele):
                pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def get_goals(self) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition that are goals

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

        mongoengine_pddl_proposition = mongoengine_pddl_proposition_model.objects(
            is_goal=True)

        pddl_dto_proposition_list = []

        for ele in mongoengine_pddl_proposition:
            if self._check_mongoengine_pddl_proposition_is_correct(ele):
                pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def get_all(self) -> List[PddlDtoProposition]:
        """ get all PddlDtoProposition

        Returns:
            List[PddlDtoProposition]: list of PddlDtoProposition
        """

        mongoengine_pddl_proposition = mongoengine_pddl_proposition_model.objects()

        pddl_dto_proposition_list = []

        for ele in mongoengine_pddl_proposition:
            if self._check_mongoengine_pddl_proposition_is_correct(ele):
                pddl_dto_proposition_list.append(self._mongoengine_to_dto(ele))

        return pddl_dto_proposition_list

    def _save(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ save a PddlDtoProposition
            if the PddlDtoProposition is already saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_proposition):
            return False

        if not self._check_pddl_dto_proposition_is_correct(pddl_dto_proposition):
            return False

       # propagating saving
        result = self._mongoengine_pddl_dao_predicate.save(
            pddl_dto_proposition.get_pddl_predicate())
        if not result:
            return False
        for pddl_dto_object in pddl_dto_proposition.get_pddl_objects_list():
            result = self._mongoengine_pddl_dao_object.save(pddl_dto_object)
            if not result:
                return False

        mongoengine_pddl_proposition = self._dto_to_mongoengine(
            pddl_dto_proposition)

        if mongoengine_pddl_proposition:
            mongoengine_pddl_proposition.save()
            return True

        else:
            return False

    def _update(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ update a PddlDtoProposition
            if the PddlDtoProposition is not saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to update

        Returns:
            bool: succeed
        """

        if not self._check_pddl_dto_proposition_is_correct(pddl_dto_proposition):
            return False

        mongoengine_pddl_proposition = self._get_mongoengine(
            pddl_dto_proposition)

        # check if proposition exists
        if mongoengine_pddl_proposition:
            new_pddl_proposition_mongoengine = self._dto_to_mongoengine(
                pddl_dto_proposition)

            if new_pddl_proposition_mongoengine:
                mongoengine_pddl_proposition.PddlPredicateModel = new_pddl_proposition_mongoengine.PddlPredicateModel
                mongoengine_pddl_proposition.pddl_objects = new_pddl_proposition_mongoengine.pddl_objects
                mongoengine_pddl_proposition.is_goal = new_pddl_proposition_mongoengine.is_goal
                mongoengine_pddl_proposition.save()
            else:
                return False

            return True

        else:
            return False

    def save(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ save or update a PddlDtoProposition
            if the PddlDtoProposition is not saved it will be saved, else it will be updated

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_dto_proposition):
            return self._update(pddl_dto_proposition)

        else:
            return self._save(pddl_dto_proposition)

    def delete(self, pddl_dto_proposition: PddlDtoProposition) -> bool:
        """ delete a PddlDtoProposition
            if the PddlDtoProposition is not saved return False, else return True

        Args:
            pddl_dto_proposition (PddlDtoProposition): PddlDtoProposition to delete

        Returns:
            bool: succeed
        """

        mongoengine_pddl_proposition = self._get_mongoengine(
            pddl_dto_proposition)

        # check if proposition exists
        if mongoengine_pddl_proposition:
            mongoengine_pddl_proposition.delete()
            return True

    def delete_all(self) -> bool:
        """ delete all pddl propositions

        Returns:
            bool: succeed
        """

        pddl_dto_proposition_list = self.get_all()

        for ele in pddl_dto_proposition_list:
            self.delete(ele)

        return True
