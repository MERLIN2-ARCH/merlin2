
""" Pddl Predicate Dao Interface """

from abc import abstractmethod
from pddl_dao.pddl_dto.pddl_predicate_dto import PddlPredicateDto
from pddl_dao.pddl_dao_interface.generic_pddl_dao import GenericPddlDao


class PddlPredicateDao(GenericPddlDao):
    """ Pddl Predicate Dao Abstract Class """

    @abstractmethod
    def get(self, predicate_name: str) -> PddlPredicateDto:
        """ get a PddlPredicateDto with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto of the pddl predicate name
        """
