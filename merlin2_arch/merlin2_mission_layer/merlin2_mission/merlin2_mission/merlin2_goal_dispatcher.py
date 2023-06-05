# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" Goal Dispatcher, which is in chager of adding/deleting new propositions
    as goals and executing/canceling the Executor (canceling everything else)
"""

from typing import List

from kant_dto import PddlPropositionDto
from kant_dao import ParameterLoader
from kant_dao.dao_factory.dao_factories.dao_factory import DaoFactory

from merlin2_msgs.action import Execute

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

    def get_dao_factory(self) -> DaoFactory:
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
            self.__node.get_logger().info("Saving goals")
            succeed = self.__pddl_proposition_dao.save(pddl_proposition_dto)
            self.__node.get_logger().info("Saving goals: " + str(succeed))

            if not succeed:
                return False
        self.__node.get_logger().info("goals saved")

        # call executor
        goal = Execute.Goal()
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__node.get_logger().info("wait for results")
        self.__action_client.wait_for_result()

        self.result = self.__action_client.get_result()
        self.__node.get_logger().info("results got")
        # results
        succeed = (self.__action_client.is_succeeded() and
                   self.result.generate_pddl and
                   self.result.generate_plan and
                   self.result.dispatch_plan)

        return succeed

    def get_result(self) -> Execute.Result:
        """ get result of the goal execution """

        return self.result

    def cancel_goals(self):
        """ cancel executor (canceling everything else)
            and delete current goals
        """

        self.__action_client.cancel_goal()
