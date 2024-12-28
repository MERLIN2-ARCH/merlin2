#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta

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


""" Merlin2 Plan Dispatcher Node """

import time
from typing import Dict

from merlin2_msgs.action import (
    DispatchPlan,
    DispatchAction
)

import rclpy

from kant_dto import (
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlObjectDto
)

from kant_dao import ParameterLoader

from simple_node import Node


class Merlin2PlanDispatcherNode(Node):
    """ Merlin2 Plan Dispatcher Node Class """

    def __init__(self):

        super().__init__("plan_dispatcher_node", namespace="merlin2")

        # loading parameters
        parameter_loader = ParameterLoader(self)
        dao_factory = parameter_loader.get_dao_factory()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()
        self.pddl_action_dao = dao_factory.create_pddl_action_dao()
        self.pddl_object_dao = dao_factory.create_pddl_object_dao()

        # action server/client
        self.__action_client = None
        self.__action_server = self.create_action_server(
            DispatchPlan,
            "dispatch_plan",
            execute_callback=self.__execute_server,
            cancel_callback=self.__cancel_callback
        )

    def __cancel_callback(self):
        if self.__action_server.is_working():
            while self.__action_client is None:
                time.sleep(0.01)
            self.__action_client.cancel_goal()

    def _call_action(self, goal: DispatchAction.Goal):
        self.__action_client = self.create_action_client(
            DispatchAction, goal.action.action_name)
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

    def __execute_server(self, goal_handle) -> DispatchPlan.Result:
        """ action server execute callback """

        result = DispatchPlan.Result()

        for action in goal_handle.request.plan:
            self.get_logger().info("Executing action " + str(action.action_name) +
                                   " with objects " + str(action.objects))

            pddl_action_dto = self.pddl_action_dao.get(action.action_name)

            if pddl_action_dto is None:
                goal_handle.abort()
                return result

            pddl_parameter_dto_list = pddl_action_dto.get_parameters()
            pddl_efect_dto_list = pddl_action_dto.get_effects()
            pddl_objects_dto_dict = {}

            for object_name, pddl_parameter_dto in zip(action.objects, pddl_parameter_dto_list):
                pddl_object_dto = self.pddl_object_dao.get(object_name)
                pddl_objects_dto_dict[pddl_parameter_dto.get_name(
                )] = pddl_object_dto

            # creating action goal
            goal = DispatchAction.Goal()
            goal.action = action

            # checking durative
            if pddl_action_dto.get_durative():
                pddl_effect_dict = {}

                # clasifaying effects by time
                for pddl_effect_dto in pddl_efect_dto_list:

                    if pddl_effect_dto.get_time() not in pddl_effect_dict:
                        pddl_effect_dict[pddl_effect_dto.get_time()] = []

                    pddl_effect_dict[pddl_effect_dto.get_time()
                                     ].append(pddl_effect_dto)

                # before calling action
                at_start = PddlConditionEffectDto.AT_START
                if at_start in pddl_effect_dict:
                    for pddl_effect_dto in pddl_effect_dict[at_start]:

                        if not self.__apply_effect(pddl_effect_dto, pddl_objects_dto_dict):
                            goal_handle.abort()
                            return result

                # over calling action
                over_all = PddlConditionEffectDto.OVER_ALL
                if over_all in pddl_effect_dict:
                    for pddl_effect_dto in pddl_effect_dict[over_all]:

                        if not self.__apply_effect(pddl_effect_dto, pddl_objects_dto_dict):
                            goal_handle.abort()
                            return result

                # calling action
                self._call_action(goal)
                self.get_logger().info("Action " + str(action.action_name) + " finished")

                # over calling action
                if over_all in pddl_effect_dict:
                    for pddl_effect_dto in pddl_effect_dict[over_all]:
                        pddl_effect_dto.set_is_negative(
                            not pddl_effect_dto.get_is_negative())
                        if not self.__apply_effect(pddl_effect_dto, pddl_objects_dto_dict):
                            goal_handle.abort()
                            return result

                if self.__action_client.is_canceled():
                    break

                # after calling action
                at_end = PddlConditionEffectDto.AT_END
                if at_end in pddl_effect_dict:
                    for pddl_effect_dto in pddl_effect_dict[at_end]:

                        if not self.__apply_effect(pddl_effect_dto, pddl_objects_dto_dict):
                            goal_handle.abort()
                            return result

                if goal_handle.is_cancel_requested:
                    break

            else:

                # calling action
                self._call_action(goal)

                if self.__action_client.is_canceled():
                    break

                for pddl_effect_dto in pddl_efect_dto_list:
                    if not self.__apply_effect(pddl_effect_dto, pddl_objects_dto_dict):
                        goal_handle.abort()
                        return result

                if goal_handle.is_cancel_requested:
                    break

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.__action_client = None
            return result

        elif self.__action_client and not self.__action_client.is_succeeded():
            goal_handle.abort()
            self.__action_client = None
            return result

        # reset action client
        goal_handle.succeed()
        self.__action_client = None

        return result

    def __build_proposition(
        self,
        pddl_effect_dto: PddlPropositionDto,
        pddl_objects_dto_dict: Dict[str, PddlObjectDto],
    ) -> PddlPropositionDto:
        """ create a pddl proposition from an action effect

        Args:
            pddl_effect_dto (PddlPropositionDto): action effect
            pddl_objects_dto_dict (Dict[str, PddlObjectDto]): dict of objects used by the action

        Returns:
            PddlPropositionDto: proposition created
        """

        proposition_objects_dto_list = []

        for pdll_parameter_dto in pddl_effect_dto.get_objects():
            proposition_objects_dto_list.append(
                pddl_objects_dto_dict[pdll_parameter_dto.get_name()])

        pddl_proposition_dto = PddlPropositionDto(
            pddl_effect_dto.get_predicate(), proposition_objects_dto_list)

        return pddl_proposition_dto

    def __update_knowledge(
            self,
            pddl_proposition_dto: PddlPropositionDto,
            deleted: bool
    ) -> bool:
        """ update knowledge using a ppdl proposition

        Args:
            pddl_proposition_dto (PddlPropositionDto): proposition to update
            deleted (bool): check if proposition must be deleted

        Returns:
            bool: succeed if dao succeed
        """
        succeed = True

        if deleted:
            succeed = self.pddl_proposition_dao.delete(
                pddl_proposition_dto)
        else:
            succeed = self.pddl_proposition_dao.save(
                pddl_proposition_dto)

        return succeed

    def __apply_effect(
        self,
        pddl_effect_dto: PddlConditionEffectDto,
        pddl_objects_dto_dict: Dict[str, PddlObjectDto]
    ) -> bool:
        """ apply an action effect removing or adding knowledge

        Args:
            pddl_effect_dto (PddlConditionEffectDto): effect to apply
            pddl_objects_dto_dict (Dict[str, PddlObjectDto]): objects of the action of the effect

        Returns:
            bool: succeed
        """

        pddl_proposition_dto = self.__build_proposition(
            pddl_effect_dto, pddl_objects_dto_dict)

        succeed = self.__update_knowledge(
            pddl_proposition_dto, pddl_effect_dto.get_is_negative())

        return succeed


def main():
    rclpy.init()
    node = Merlin2PlanDispatcherNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
