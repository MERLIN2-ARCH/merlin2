
""" Goal Dispatcher, which is in chager of adding new propositions
    as goals and executing/canceling the Executor (canceling everything else)
"""

from typing import List

from pddl_dto import PddlPropositionDto

from pddl_dao import PddlDaoParameterLoader

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory

from merlin2_arch_interfaces.action import Execute

from custom_ros2 import Node


class Merlin2GoalDispatcher:
    """ MERLIN2 Goal Dispatcher Class """

    def __init__(self, node: Node):
        self.__node = node

        # loading parameters
        pddl_dao_parameter_loader = PddlDaoParameterLoader(self.__node)
        self.__pddl_dao_factory = pddl_dao_parameter_loader.get_pddl_dao_factory()
        self.__pddl_proposition_dao = self.__pddl_dao_factory.create_pddl_proposition_dao()

        # action client
        self.__action_client = self.__node.create_action_client(
            Execute, "execute")

    def get_pddl_factory(self) -> PddlDaoFactory:
        """ get pddl dao factory of the goal dispatcher

        Returns:
            PddlDaoFactory: pddl dao factory
        """

        return self.__pddl_dao_factory

    def execute_goals(self, pddl_proposition_dto_list: List[PddlPropositionDto]) -> bool:
        """ add goals to knowledge base and call executor

        Args:
            pddl_proposition_dto_list (List[PddlPropositionDto]): list of goals to add

        Returns:
            bool: succeed?
        """

        succeed = True

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

        results = self.__action_client.get_result()

        # results
        succeed = (self.__action_client.is_succeeded() and
                   results.dispatch_plan and
                   results.generate_pddl and
                   results.dispatch_plan)

        return succeed

    def cancel_goals(self):
        """ cancel executor (canceling everything else) """

        self.__action_client.cancel_goal()
