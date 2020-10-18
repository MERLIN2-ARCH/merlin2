
""" Pddl Action Dao Interface """

from abc import abstractmethod
from pddl_dao.pddl_dto.pddl_action_dto import PddlActionDto
from pddl_dao.pddl_dao_interface.pddl_dao import PddlDao


class PddlActionDao(PddlDao):
    """ Pddl Action Dao Abstract Class """

    @abstractmethod
    def get(self, action_name: str) -> PddlActionDto:
        """ get a PddlActionDto with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlActionDto: PddlActionDto of the pddl action name
        """
