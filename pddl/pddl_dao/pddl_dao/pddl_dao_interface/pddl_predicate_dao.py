
""" Pddl Predicate Dao Interface """

from abc import abstractmethod
from pddl_dto import PddlPredicateDto
from pddl_dao.pddl_dao_interface import PddlDao


class PddlPredicateDao(PddlDao):
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
