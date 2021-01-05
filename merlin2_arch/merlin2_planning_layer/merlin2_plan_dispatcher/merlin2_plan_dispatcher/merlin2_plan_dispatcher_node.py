
""" Merlin2 Plan Dispatcher Node """

from typing import Dict
import time
from merlin2_arch_interfaces.action import (
    DispatchPlan,
    DispatchAction
)

import rclpy
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

from custom_ros2 import (
    Node,
    ActionSingleServer,
    ActionClient
)


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

        # action server/client
        self.__action_client = None
        self.__action_server = ActionSingleServer(self,
                                                  DispatchPlan,
                                                  "dispatch_plan",
                                                  execute_callback=self.__execute_server,
                                                  cancel_callback=self.__cancel_callback)

    def destroy(self):
        """ destroy node method """

        if self.__action_client:
            self.__action_server.destroy()
        super().destroy_node()

    def __cancel_callback(self):
        if self.__action_client:
            if self.__action_client.is_working():
                self.__action_client.cancel_goal()

    def _call_action(self, goal):
        self.__action_client = ActionClient(
            self, DispatchAction, goal.action.action_name)
        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

    def __execute_server(self, goal_handle):
        """ action server execute callback """

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

                # calling action
                self._call_action(goal)

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

                # calling action
                self._call_action(goal)

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
            if not self.__action_client:
                goal_handle.succeed()
            elif self.__action_client.is_succeeded():
                goal_handle.succeed()
            else:
                goal_handle.abort()

        # reset action client
        self.__action_client = None

        return result

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
