
""" Merlin2 Action Base """

from typing import List
from abc import ABC, abstractmethod

from merlin2_arch_interfaces.action import DispatchAction
from merlin2_arch_interfaces.msg import PlanAction

from kant_dto import PddlActionDto, PddlConditionEffectDto, PddlObjectDto
from kant_dao import ParameterLoader

from simple_node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class Merlin2Action(Node, PddlActionDto, ABC):
    """ Merlin2 Action Class """

    def __init__(self, a_name, durative: bool = True):

        Node.__init__(self, a_name, namespace="merlin2")
        PddlActionDto.__init__(self,
                               a_name,
                               durative=durative)

        # loading parameters
        parameter_loader = ParameterLoader(self)
        dao_factory = parameter_loader.get_dao_factory()
        self.__pddl_action_dao = dao_factory.create_pddl_action_dao()

        # creating and saving the action
        pddl_parameter_dto_list = self.create_parameters()
        pddl_effect_dto_list = self.create_efects()
        pddl_condition_dto_list = self.create_conditions()

        self.set_parameters(pddl_parameter_dto_list)
        self.set_effects(pddl_effect_dto_list)
        self.set_conditions(pddl_condition_dto_list)

        succeed = self.save_action()

        if not succeed:
            raise Exception("Wrong Action: " + str(self))

        # action
        self.__action_server = self.create_action_server(DispatchAction,
                                                         a_name,
                                                         self.__execute_server,
                                                         cancel_callback=self.__cancel_callback)

    def set_parameters(self, parameters):
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

    def get_parameters(self, names: List[str] = None):
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

    def destroy_node(self):
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
    def cancel_action(self):
        """ Code to cancel the action. Must be implemented. """

    @abstractmethod
    def create_parameters(self) -> List[PddlObjectDto]:
        """ Code to the parameters of the action. Must be implemented.

        Returns:
            List[PddlObjectDto]: list of parameters
        """

    @abstractmethod
    def create_efects(self) -> List[PddlConditionEffectDto]:
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

        succeed = self.__pddl_action_dao.save(self)
        return succeed

    def __cancel_callback(self):
        self.cancel_action()

    def __execute_server(self, goal_handle):
        result = DispatchAction.Result()

        succeed = self.run_action(goal_handle.request.action)

        if self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled()

        else:
            if succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        return result
