
""" Goal Dispatcher, which is in chager of adding/deleting new propositions
    as goals and executing/canceling the Executor (canceling everything else)
"""

from typing import List

from kant_dto import PddlPropositionDto
from kant_dao import ParameterLoader
from kant_dao.dao_factory.dao_factories.dao_factory import DaoFactory

from merlin2_arch_interfaces.action import Execute

from simple_node import Node


class Merlin2GoalDispatcher:
    """ MERLIN2 Goal Dispatcher Class """

    def __init__(self, node: Node):
        self.__node = node

        self.result = None

        # loading parameters
        parameter_loader = ParameterLoader(self.__node)
        self.__dao_factory = parameter_loader.get_dao_factory()
        self.__pddl_proposition_dao = self.__dao_factory.create_pddl_proposition_dao()

        # action client
        self.__action_client = self.__node.create_action_client(
            Execute, "execute")

    def get_pddl_factory(self) -> DaoFactory:
        """ get pddl dao factory of the goal dispatcher

        Returns:
            DaoFactory: pddl dao factory
        """

        return self.__dao_factory

    def execute_goals(self, pddl_proposition_dto_list: List[PddlPropositionDto]) -> bool:
        """ add goals to knowledge base and call executor

        Args:
            pddl_proposition_dto_list (List[PddlPropositionDto]): list of goals to add

        Returns:
            bool: succeed?
        """

        succeed = True
        self.result = None

        # save goals
        for pddl_proposition_dto in pddl_proposition_dto_list:
            pddl_proposition_dto.set_is_goal(True)
            succeed = self.__pddl_proposition_dao.save(pddl_proposition_dto)

            if not succeed:
                return False

        # call executor
        goal = Execute.Goal()
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        self.results = self.__action_client.get_result()

        # results
        succeed = (self.__action_client.is_succeeded() and
                   self.results.generate_pddl and
                   self.results.generate_plan and
                   self.results.dispatch_plan)

        return succeed

    def get_result(self) -> Execute.Result:
        """ get result of the goal execution """

        return self.result

    def cancel_goals(self):
        """ cancel executor (canceling everything else)
            and delete current goals
        """

        self.__action_client.cancel_goal()

        for pddl_goal_dto in self.__pddl_proposition_dao.get_goals():
            self.__pddl_proposition_dao.delete(pddl_goal_dto)
