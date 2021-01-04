
""" Merlin2 Plan Dispatcher Node """

import threading
import collections

from typing import Dict

from merlin2_plan_sys_interfaces.action import (
    DispatchPlan,
    DispatchAction
)

import rclpy
from rclpy.action import ActionServer, CancelResponse
from action_msgs.msg import GoalStatus


from pddl_dto import (
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlObjectDto
)

from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies
)

from threaded_node.node import Node


class Merlin2PlanDispatcherNode(Node):
    """ Merlin2 Plan Dispatcher Node Class """

    def __init__(self):

        super().__init__("merlin2_plan_dispatcher_mode")

        # param names
        pddl_dao_family_param_name = "pddl_dao_family"
        mongoengine_uri_param_name = "mongoengine_uri"

        # declaring params
        self.declare_parameter(pddl_dao_family_param_name,
                               PddlDaoFamilies.MONGOENGINE)
        self.declare_parameter(mongoengine_uri_param_name,
                               "mongodb://localhost:27017/merlin2")

        # getting params
        pddl_dao_family = self.get_parameter(
            pddl_dao_family_param_name).get_parameter_value().integer_value
        mongoengine_uri = self.get_parameter(
            mongoengine_uri_param_name).get_parameter_value().string_value

        # creating pddl daos
        pddl_dao_factory = PddlDaoFactoryFactory().create_pddl_dao_factory(
            pddl_dao_family, uri=mongoengine_uri, node=self)
        self.pddl_proposition_dao = pddl_dao_factory.create_pddl_proposition_dao()
        self.pddl_action_dao = pddl_dao_factory.create_pddl_action_dao()
        self.pddl_object_dao = pddl_dao_factory.create_pddl_object_dao()

        # action vars
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        # action server
        self._action_server = ActionServer(self,
                                           DispatchPlan,
                                           "dispatch_plan",
                                           execute_callback=self.__execute_server,
                                           cancel_callback=self.__cancel_server,
                                           handle_accepted_callback=self.__accepted_callback,
                                           )

    def destroy(self):
        """ destroy node method
        """

        self._action_server.destroy()
        super().destroy_node()

    def __accepted_callback(self, goal_handle):
        """ action server accepted callback or defer execution of an already accepted goal

        Args:
            goal_handle: goal handle
        """

        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info("Goal put in the queue")
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __cancel_server(self, goal_handle):
        """ action server cancel callback

        Args:
            goal_handle: goal handle

        Returns:
            CancelResponse: cancel response
        """

        self.get_logger().info("Cancelling Plan Dispatcher")
        return CancelResponse.ACCEPT

    def __execute_server(self, goal_handle):
        """action server execute callback"""

        try:

            result = DispatchPlan.Result()

            succeed = True

            for action in goal_handle.request.plan:
                self.get_logger().info("Executing action " + str(action.action_name) +
                                       " with objects " + str(action.objects))

                pddl_action_dto = self.pddl_action_dao.get(action.action_name)
                pddl_parameter_dto_list = pddl_action_dto.get_pddl_parameters_list()
                pddl_efect_dto_list = pddl_action_dto.get_pddl_effects_list()
                pddl_objects_dto_dict = {}

                for object_name, pddl_parameter_dto in zip(action.objects, pddl_parameter_dto_list):
                    pddl_object_dto = self.pddl_object_dao.get(object_name)
                    pddl_objects_dto_dict[pddl_parameter_dto.get_object_name(
                    )] = pddl_object_dto

                # checking durative
                if pddl_action_dto.get_durative():
                    pddl_effect_dict = {}

                    for pddl_effect_dto in pddl_efect_dto_list:

                        if pddl_effect_dto.get_time() not in pddl_effect_dict:
                            pddl_effect_dict[pddl_effect_dto.get_time()] = []

                        pddl_effect_dict[pddl_effect_dto.get_time()
                                         ].append(pddl_effect_dto)

                    # before calling action
                    at_start = PddlConditionEffectDto.AT_START
                    if at_start in pddl_effect_dict:
                        for pddl_effect_dto in pddl_effect_dict[at_start]:
                            pddl_proposition_dto = self.__build_proposition(
                                pddl_effect_dto, pddl_objects_dto_dict)
                            succeed = self.__update_knowledge(
                                pddl_proposition_dto, pddl_effect_dto.get_is_negative())

                        if not succeed:
                            goal_handle.abort()
                            return result

                    over_all = PddlConditionEffectDto.OVER_ALL
                    if over_all in pddl_effect_dict:
                        for pddl_effect_dto in pddl_effect_dict[over_all]:
                            pddl_proposition_dto = self.__build_proposition(
                                pddl_effect_dto, pddl_objects_dto_dict)
                            succeed = self.__update_knowledge(
                                pddl_proposition_dto, pddl_effect_dto.get_is_negative())

                        if not succeed:
                            goal_handle.abort()
                            return result

                    #############################

                    # TO DO
                    # Call action

                    #############################

                    # after calling action
                    if over_all in pddl_effect_dict:
                        for pddl_effect_dto in pddl_effect_dict[over_all]:
                            pddl_proposition_dto = self.__build_proposition(
                                pddl_effect_dto, pddl_objects_dto_dict)
                            succeed = self.__update_knowledge(
                                pddl_proposition_dto, not pddl_effect_dto.get_is_negative())

                        if not succeed:
                            goal_handle.abort()
                            return result

                    at_end = PddlConditionEffectDto.AT_END
                    if at_end in pddl_effect_dict:
                        for pddl_effect_dto in pddl_effect_dict[at_end]:
                            pddl_proposition_dto = self.__build_proposition(
                                pddl_effect_dto, pddl_objects_dto_dict)
                            succeed = self.__update_knowledge(
                                pddl_proposition_dto, pddl_effect_dto.get_is_negative())

                        if not succeed:
                            goal_handle.abort()
                            return result

                else:

                    #############################

                    # TO DO
                    # Call action

                    #############################

                    for pddl_effect_dto in pddl_efect_dto_list:
                        pddl_proposition_dto = self.__build_proposition(
                            pddl_effect_dto, pddl_objects_dto_dict)
                        succeed = self.__update_knowledge(
                            pddl_proposition_dto, pddl_effect_dto.get_is_negative())

                        if not succeed:
                            goal_handle.abort()
                            return result

            if(goal_handle.status == GoalStatus.STATUS_CANCELED and
               goal_handle.status == GoalStatus.STATUS_CANCELING):
                goal_handle.canceled()

            else:
                goal_handle.succeed()

            return result

        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info("Next goal pulled from the queue")
                    self._current_goal.execute()

                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None

    def __build_proposition(self,
                            pddl_effect_dto: PddlPropositionDto,
                            pddl_objects_dto_dict: Dict[str, PddlObjectDto],
                            ) -> PddlPropositionDto:
        """ create a pddl proposition from am action effect

        Args:
            pddl_effect_dto (PddlPropositionDto): action effect
            pddl_objects_dto_dict (Dict[str, PddlObjectDto]): dict of objects used by the action

        Returns:
            PddlPropositionDto: proposition created
        """

        proposition_objects_dto_list = []

        for pdll_parameter_dto in pddl_effect_dto.get_pddl_objects_list():
            proposition_objects_dto_list.append(
                pddl_objects_dto_dict[pdll_parameter_dto.get_object_name()])

        pddl_proposition_dto = PddlPropositionDto(
            pddl_effect_dto.get_pddl_predicate(), proposition_objects_dto_list)

        return pddl_proposition_dto

    def __update_knowledge(self,
                           pddl_proposition_dto: PddlPropositionDto,
                           deleted: bool) -> bool:
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


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PlanDispatcherNode()

    node.join_spin()

    node.destroy()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
