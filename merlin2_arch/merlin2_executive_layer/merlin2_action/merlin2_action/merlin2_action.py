
""" Merlin2 Action Base """

from typing import List
import time
from merlin2_arch_interfaces.action import DispatchAction
from merlin2_arch_interfaces.msg import PlanAction
from pddl_dto import PddlActionDto, PddlConditionEffectDto, PddlObjectDto
from pddl_dao import PddlDaoParameterLoader
from custom_ros2 import (
    Node,
    ActionSingleServer
)


class Merlin2Action(Node, PddlActionDto):
    """ Merlin2 Action Class """

    def __init__(self, a_name, durative: bool = True, duration: int = 10):

        Node.__init__(self, a_name)
        PddlActionDto.__init__(self,
                               a_name,
                               durative=durative,
                               duration=duration)

        # loading parameters
        pddl_dao_parameter_loader = PddlDaoParameterLoader(self)
        pddl_dao_factory = pddl_dao_parameter_loader.get_pddl_dao_factory()
        self.__pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()

        # creating and saving the action
        pddl_parameter_dto_list = self.create_parameters()
        pddl_effect_dto_list = self.create_efects()
        pddl_condition_dto_list = self.create_conditions()

        self.set_pddl_parameters_list(pddl_parameter_dto_list)
        self.set_pddl_effects_list(pddl_effect_dto_list)
        self.set_pddl_conditions_list(pddl_condition_dto_list)

        self.save_action()

        # action
        self._is_canceled = False
        self.__action_server = ActionSingleServer(self, DispatchAction,
                                                  "merlin2/" + a_name,
                                                  self.__execute_server,
                                                  cancel_callback=self.__cancel_callback)

    def __hash__(self):
        return Node.__hash__(self)

    def destroy(self):
        """ destroy node method """

        self.__action_server.destroy()
        super().destroy_node()

    def run_action(self, goal: PlanAction) -> bool:
        """ Code of the action. Must be implemented.

        Returns:
            bool: action succeed
        """

        return NotImplementedError()

    def cancel_action(self):
        """ Code to cancel the action. Must be implemented. """

        return NotImplementedError()

    def create_parameters(self) -> List[PddlObjectDto]:
        """ Code to the parameters of the action. Must be implemented.

        Returns:
            List[PddlObjectDto]: list of parameters
        """

        return NotImplementedError()

    def create_efects(self) -> List[PddlConditionEffectDto]:
        """ Code to the efects of the action. Must be implemented.

        Returns:
            List[PddlObjectDto]: list of parameters
        """

        return NotImplementedError()

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        """ Code to the conditions of the action. Must be implemented.

        Returns:
            List[PddlObjectDto]: list of parameters
        """

        return NotImplementedError()

    def save_action(self) -> bool:
        """ save/update action

        Returns:
            bool: succeedd
        """

        succeed = self.__pddl_action_dao.save(self)
        return succeed

    def get_is_canceled(self) -> bool:
        """ get if action is canceled

        Returns:
            bool: action is canceled?
        """

        return self._is_canceled

    def __cancel_callback(self):
        self._is_canceled = True
        self.cancel_action()

    def __execute_server(self, goal_handle):
        self._is_canceled = False
        result = DispatchAction.Result()

        succeed = self.run_action(goal_handle.request.action)

        if self._is_canceled:
            while not goal_handle.is_cancel_requested:
                time.sleep(0.05)
            goal_handle.canceled()

        else:
            if succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        return result
