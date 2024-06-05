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


""" Merlin2 Action Base """

from typing import List, Union
from abc import ABC, abstractmethod

from merlin2_msgs.action import DispatchAction
from merlin2_msgs.msg import PlanAction

from kant_dto import PddlActionDto, PddlConditionEffectDto, PddlObjectDto
from kant_dao import ParameterLoader

from simple_node import Node
from rclpy.parameter import Parameter


class Merlin2Action(Node, PddlActionDto, ABC):
    """ Merlin2 Action Class """

    def __init__(self, a_name: str, durative: bool = True) -> None:

        Node.__init__(self, a_name, namespace="merlin2")
        PddlActionDto.__init__(self,
                               a_name,
                               durative=durative)

        # loading parameters
        parameter_loader = ParameterLoader(self)
        self.dao_factory = parameter_loader.get_dao_factory()
        self.__pddl_action_dao = self.dao_factory.create_pddl_action_dao()

        # creating and saving the action
        pddl_parameter_dto_list = self.create_parameters()
        pddl_effect_dto_list = self.create_effects()
        pddl_condition_dto_list = self.create_conditions()

        self.set_parameters(pddl_parameter_dto_list)
        self.set_effects(pddl_effect_dto_list)
        self.set_conditions(pddl_condition_dto_list)

        if not self.save_action():
            raise Exception("Wrong PDDL action: " + a_name)

        # action
        self.__action_server = self.create_action_server(DispatchAction,
                                                         a_name,
                                                         self.__execute_server,
                                                         cancel_callback=self.__cancel_callback)

    def set_parameters(self, parameters: Union[PddlActionDto | Parameter]) -> None:
        """ set parameters for PddlActionDto and Node
            both has the same method

        Args:
            parameters (PddlObjectDto || Parameter): list of parameters that can be PddlObjectDto or Parameter

        Returns:
            None || List[SetParametersResult]: returns None if Dto version or SetParametersResult if Node version
        """

        if parameters:
            if isinstance(parameters, list):

                if isinstance(parameters[0], PddlObjectDto):
                    PddlActionDto.set_parameters(self, parameters)

                elif isinstance(parameters[0], Parameter):
                    return Node.set_parameters(self, parameters)

    def get_parameters(self, names: List[str] = None) -> None:
        """ get parameters for PddlActionDto and Node
            both has the same method

        Args:
            names (List[str], optional): names of parameters for a Node. 
                                         Only used with Node version. Defaults to None.

        Returns:
            List[PddlObjectDto] || List[Parameter]: returns a list of PddlObjectDto 
                                                   if Dto version or Parameters if Node version
        """

        if not names:
            return PddlActionDto.get_parameters(self)

        else:
            return Node.get_parameters(self, names)

    def __hash__(self):
        return Node.__hash__(self)

    def destroy_node(self) -> None:
        """ destroy node method """

        self.__pddl_action_dao.delete(self)
        super().destroy_node()

    @abstractmethod
    def run_action(self, goal: PlanAction) -> bool:
        """ Code of the action. Must be implemented.

        Returns:
            bool: action succeed
        """

    @abstractmethod
    def cancel_action(self) -> None:
        """ Code to cancel the action. Must be implemented. """

    @abstractmethod
    def create_parameters(self) -> List[PddlObjectDto]:
        """ Code to the parameters of the action. Must be implemented.

        Returns:
            List[PddlObjectDto]: list of parameters
        """

    @abstractmethod
    def create_effects(self) -> List[PddlConditionEffectDto]:
        """ Code to the efects of the action. Must be implemented.

        Returns:
            List[PddlConditionEffectDto]: list of parameters
        """

    @abstractmethod
    def create_conditions(self) -> List[PddlConditionEffectDto]:
        """ Code to the conditions of the action. Must be implemented.

        Returns:
            List[PddlConditionEffectDto]: list of parameters
        """

    def save_action(self) -> bool:
        """ save/update action

        Returns:
            bool: succeedd
        """

        return self.__pddl_action_dao.save(self)

    def __cancel_callback(self) -> None:
        self.cancel_action()

    def __execute_server(self, goal_handle) -> DispatchAction.Result:
        result = DispatchAction.Result()

        succeed = self.run_action(goal_handle.request.action)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()

        else:
            if succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        return result
