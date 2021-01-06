
""" Goal Dispatcher, which is in chager of adding new propositions
    as goals and executing/canceling the Executor (canceling everything else)
"""

from typing import List

from pddl_dto import PddlPropositionDto

from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)

from merlin2_arch_interfaces.action import Execute

from custom_ros2 import (
    Node,
    ActionClient
)


class Merlin2GoalDispatcher:
    """ MERLIN2 Goal Dispatcher Class """

    def __init__(self, node: Node):
        self.__node = node

        # param names
        pddl_dao_family_param_name = "pddl_dao_family"
        mongoengine_uri_param_name = "mongoengine_uri"

        # declaring params
        self.__node.declare_parameter(pddl_dao_family_param_name,
                                      PddlDaoFamilies.MONGOENGINE)
        self.__node.declare_parameter(mongoengine_uri_param_name,
                                      "mongodb://localhost:27017/merlin2")

        # getting params
        pddl_dao_family = self.__node.get_parameter(
            pddl_dao_family_param_name).get_parameter_value().integer_value
        mongoengine_uri = self.__node.get_parameter(
            mongoengine_uri_param_name).get_parameter_value().string_value

        # creating pddl dao
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        self.__pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            pddl_dao_family, uri=mongoengine_uri, node=self.__node)

        self.__pddl_proposition_dao = self.__pddl_dao_factory.create_pddl_proposition_dao()

        # action client
        self.__action_client = ActionClient(self.__node, Execute, "execute")

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
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        # results
        succeed = self.__action_client.is_succeeded()
        return succeed

    def cancel_goals(self):
        """ cancel executor (canceling everything else) """

        self.__action_client.cancel_goal()
