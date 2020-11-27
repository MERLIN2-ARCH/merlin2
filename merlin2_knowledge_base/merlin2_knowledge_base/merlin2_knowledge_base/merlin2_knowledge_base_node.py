
""" Merlin2 Knowledge Base Node"""

import rclpy

from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlAction,
    UpdatePddlType,
    UpdatePddlPredicate,
    UpdatePddlProposition,
    UpdatePddlObject,
    GetPddlAction,
    GetPddlType,
    GetPddlObject,
    GetPddlPredicate,
    GetPddlProposition
)
from std_srvs.srv import Empty

from threaded_node.node import Node

from merlin2_knowledge_base.merlin2_knowledge_base_parser import (
    DtoMsgParser,
    MsgDtoParser
)

from merlin2_knowledge_base.merlin2_knowledge_base import (
    Merlin2KnowledgeBase
)


class Merlin2KnowledgeBaseNode(Node):
    """ Merlin2 Knowledge Base Node Class """

    def __init__(self):

        super().__init__("merlin2_knowledge_base")

        self.knowledge_base = Merlin2KnowledgeBase()
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # types srvs
        self.create_service(
            GetPddlType, "get_types", self.get_types)

        self.create_service(
            UpdatePddlType, "update_type", self.update_type)

        self.create_service(
            Empty, "delete_all_types", self.delete_all_types)

        # objects srvs
        self.create_service(
            GetPddlObject, "get_objects", self.get_objects)

        self.create_service(
            UpdatePddlObject, "update_object", self.update_object)

        self.create_service(
            Empty, "delete_all_objects", self.delete_all_objects)

        # predicates srvs
        self.create_service(
            GetPddlPredicate, "get_predicates", self.get_predicates)

        self.create_service(
            UpdatePddlPredicate, "update_predicate", self.update_predicate)

        self.create_service(
            Empty, "delete_all_predicates", self.delete_all_predicates)

        # propositions srvs
        self.create_service(
            GetPddlProposition, "get_propositions", self.get_propositions)

        self.create_service(
            UpdatePddlProposition, "update_proposition", self.update_proposition)

        self.create_service(
            Empty, "delete_all_propositions", self.delete_all_propositions)

        # actions srvs
        self.create_service(
            GetPddlAction, "get_actions", self.get_actions)

        self.create_service(
            UpdatePddlAction, "update_action", self.update_action)

        self.create_service(
            Empty, "delete_all_actions", self.delete_all_actions)

    #### types ####

    def get_types(self,
                  req: GetPddlType.Request,
                  res: GetPddlType.Response) -> GetPddlType.Response:
        """ srv callback to get types

        Args:
            req (GetPddlType.Request): request with type name
            res (GetPddlType.Response): response with types

        Returns:
            GetPddlType.Response: response with types
        """

        if req.type_name:

            pddl_type_dto = self.knowledge_base.get_type(req.type_name)

            if pddl_type_dto:

                pddl_type_msg = self.dto_msg_parser.type_dto_to_msg(
                    pddl_type_dto)

                res.pddl_types = [pddl_type_msg]
        else:

            pddl_type_dtos = self.knowledge_base.get_all_types()

            pddl_type_msg_list = []

            for pddl_type_dto in pddl_type_dtos:
                pddl_type_msg_list.append(
                    self.dto_msg_parser.type_dto_to_msg(pddl_type_dto))

            res.pddl_types = pddl_type_msg_list

        return res

    def update_type(self,
                    req: UpdatePddlType.Request,
                    res: UpdatePddlType.Response) -> UpdatePddlType.Response:
        """ srv callbak to save or update and delete a pddl type

        Args:
            req (UpdatePddlType.Request): request with type
            res (UpdatePddlType.Response): response with success

        Returns:
            Empty.Response: response with success
        """

        succ = False

        pddl_type_dto = self.msg_dto_parser.type_msg_to_dto(req.pddl_type)

        if req.update_konwledge.update_type == UpdateKnowledge.SAVE:
            succ = self.knowledge_base.save_type(pddl_type_dto)

        elif req.update_konwledge.update_type == UpdateKnowledge.DELETE:
            succ = self.knowledge_base.delete_type(pddl_type_dto)

        res.success = succ
        return res

    def delete_all_types(self,
                         req: Empty.Request,
                         res: Empty.Response) -> Empty.Response:
        """ srv callback to delete all types

        Args:
            req (Empty.Request): empty request
            res (Empty.Response): empty response

        Returns:
            Empty.Response: empty response
        """

        self.knowledge_base.delete_all_types()

        return res

    #### objects ####

    def get_objects(self,
                    req: GetPddlObject.Request,
                    res: GetPddlObject.Response) -> GetPddlObject.Response:
        """ srv callback to get objects

        Args:
            req (GetPddlObject.Request): request with object name
            res (GetPddlObject.Response): response with objects

        Returns:
            GetPddlObject.Response: response with objects
        """

        if req.object_name:

            pddl_object_dto = self.knowledge_base.get_object(req.object_name)

            if pddl_object_dto:

                pddl_object_msg = self.dto_msg_parser.object_dto_to_msg(
                    pddl_object_dto)

                res.pddl_objects = [pddl_object_msg]
        else:

            pddl_object_dtos = self.knowledge_base.get_all_objects()

            pddl_object_msg_list = []

            for pddl_object_dto in pddl_object_dtos:
                pddl_object_msg_list.append(
                    self.dto_msg_parser.object_dto_to_msg(pddl_object_dto))

            res.pddl_objects = pddl_object_msg_list

        return res

    def update_object(self,
                      req: UpdatePddlObject.Request,
                      res: UpdatePddlObject.Response) -> UpdatePddlObject.Response:
        """ srv callbak to save or update and delete a pddl object

        Args:
            req (UpdatePddlObject.Request): request with object
            res (UpdatePddlObject.Response): response with success

        Returns:
            Empty.Response: response with success
        """

        succ = False

        pddl_object_dto = self.msg_dto_parser.object_msg_to_dto(
            req.pddl_object)

        if req.update_konwledge.update_type == UpdateKnowledge.SAVE:
            succ = self.knowledge_base.save_object(pddl_object_dto)

        elif req.update_konwledge.update_type == UpdateKnowledge.DELETE:
            succ = self.knowledge_base.delete_object(pddl_object_dto)

        res.success = succ
        return res

    def delete_all_objects(self,
                           req: Empty.Request,
                           res: Empty.Response) -> Empty.Response:
        """ srv callback to delete all objects

        Args:
            req (Empty.Request): empty request
            res (Empty.Response): empty response

        Returns:
            Empty.Response: empty response
        """

        self.knowledge_base.delete_all_objects()

        return res

    #### predicates ####

    def get_predicates(self,
                       req: GetPddlPredicate.Request,
                       res: GetPddlPredicate.Response) -> GetPddlPredicate.Response:
        """ srv callback to get predicates

        Args:
            req (GetPddlPredicate.Request): request with predicate name
            res (GetPddlPredicate.Response): response with predicates

        Returns:
            GetPddlPredicate.Response: response with predicates
        """

        if req.predicate_name:

            pddl_predicate_dto = self.knowledge_base.get_predicate(
                req.predicate_name)

            if pddl_predicate_dto:

                pddl_predicate_msg = self.dto_msg_parser.predicate_dto_to_msg(
                    pddl_predicate_dto)

                res.pddl_predicates = [pddl_predicate_msg]
        else:
            pddl_predicate_dtos = self.knowledge_base.get_all_predicates()
            pddl_predicate_msg_list = []

            for pddl_predicate_dto in pddl_predicate_dtos:
                pddl_predicate_msg_list.append(
                    self.dto_msg_parser.predicate_dto_to_msg(pddl_predicate_dto))

            res.pddl_predicates = pddl_predicate_msg_list

        return res

    def update_predicate(self,
                         req: UpdatePddlPredicate.Request,
                         res: UpdatePddlPredicate.Response) -> UpdatePddlPredicate.Response:
        """ srv callbak to save or update and delete a pddl predicate

        Args:
            req (UpdatePddlPredicate.Request): request with predicate
            res (UpdatePddlPredicate.Response): response with success

        Returns:
            Empty.Response: response with success
        """

        succ = False

        pddl_predicate_dto = self.msg_dto_parser.predicate_msg_to_dto(
            req.pddl_predicate)

        if req.update_konwledge.update_type == UpdateKnowledge.SAVE:
            succ = self.knowledge_base.save_predicate(pddl_predicate_dto)

        elif req.update_konwledge.update_type == UpdateKnowledge.DELETE:
            succ = self.knowledge_base.delete_predicate(pddl_predicate_dto)

        res.success = succ
        return res

    def delete_all_predicates(self,
                              req: Empty.Request,
                              res: Empty.Response) -> Empty.Response:
        """ srv callback to delete all predicates

        Args:
            req (Empty.Request): empty request
            res (Empty.Response): empty response

        Returns:
            Empty.Response: empty response
        """

        self.knowledge_base.delete_all_predicates()

        return res

    #### predicates ####

    def get_propositions(self,
                         req: GetPddlProposition.Request,
                         res: GetPddlProposition.Response) -> GetPddlProposition.Response:
        """ srv callback to get propositions

        Args:
            req (GetPddlProposition.Request): request with predicate name and get type
            res (GetPddlProposition.Response): response with propositions

        Returns:
            GetPddlProposition.Response: response with propositions
        """

        pddl_proposition_dtos = []
        if req.get_type == GetPddlProposition.Request.ALL:
            pddl_proposition_dtos = self.knowledge_base.get_all_propositions()

        elif req.get_type == GetPddlProposition.Request.GOALS:
            pddl_proposition_dtos = self.knowledge_base.get_propositions_goals()

        elif req.get_type == GetPddlProposition.Request.NO_GOALS:
            pddl_proposition_dtos = self.knowledge_base.get_propositions_no_goals()

        elif req.get_type == GetPddlProposition.Request.BY_PREDICATE:
            pddl_proposition_dtos = self.knowledge_base.get_propositions(
                req.predicate_name)

        pddl_predicate_msg_list = []

        for pddl_proposition_dto in pddl_proposition_dtos:
            pddl_predicate_msg_list.append(
                self.dto_msg_parser.proposition_dto_to_msg(pddl_proposition_dto))

        res.pddl_propositions = pddl_predicate_msg_list

        return res

    def update_proposition(self,
                           req: UpdatePddlProposition.Request,
                           res: UpdatePddlProposition.Response) -> UpdatePddlProposition.Response:
        """ srv callbak to save or update and delete a pddl proposition

        Args:
            req (UpdatePddlProposition.Request): request with proposition
            res (UpdatePddlProposition.Response): response with success

        Returns:
            Empty.Response: response with success
        """

        succ = False

        pddl_proposition_dto = self.msg_dto_parser.proposition_msg_to_dto(
            req.pddl_proposition)

        if req.update_konwledge.update_type == UpdateKnowledge.SAVE:
            succ = self.knowledge_base.save_proposition(pddl_proposition_dto)

        elif req.update_konwledge.update_type == UpdateKnowledge.DELETE:
            succ = self.knowledge_base.delete_proposition(pddl_proposition_dto)

        res.success = succ
        return res

    def delete_all_propositions(self,
                                req: Empty.Request,
                                res: Empty.Response) -> Empty.Response:
        """ srv callback to delete all propositions

        Args:
            req (Empty.Request): empty request
            res (Empty.Response): empty response

        Returns:
            Empty.Response: empty response
        """

        self.knowledge_base.delete_all_propositions()

        return res

    #### actions ####

    def get_actions(self,
                    req: GetPddlAction.Request,
                    res: GetPddlAction.Response) -> GetPddlAction.Response:
        """ srv callback to get types

        Args:
            req (GetPddlAction.Request): request with action name
            res (GetPddlAction.Response): response with actions

        Returns:
            GetPddlAction.Response: response with actions
        """

        if req.action_name:

            pddl_action_dto = self.knowledge_base.get_action(req.action_name)

            if pddl_action_dto:

                pddl_action_msg = self.dto_msg_parser.action_dto_to_msg(
                    pddl_action_dto)

                res.pddl_actions = [pddl_action_msg]
        else:

            pddl_action_dtos = self.knowledge_base.get_all_actions()

            pddl_action_msg_list = []

            for pddl_action_dto in pddl_action_dtos:
                pddl_action_msg_list.append(
                    self.dto_msg_parser.action_dto_to_msg(pddl_action_dto))

            res.pddl_actions = pddl_action_msg_list

        return res

    def update_action(self,
                      req: UpdatePddlAction.Request,
                      res: UpdatePddlAction.Response) -> UpdatePddlAction.Response:
        """ srv callbak to save or update and delete a pddl action

        Args:
            req (UpdatePddlAction.Request): request with action
            res (UpdatePddlAction.Response): response with success

        Returns:
            Empty.Response: response with success
        """

        succ = False

        pddl_action_dto = self.msg_dto_parser.action_msg_to_dto(
            req.pddl_action)

        if req.update_konwledge.update_type == UpdateKnowledge.SAVE:
            succ = self.knowledge_base.save_action(pddl_action_dto)

        elif req.update_konwledge.update_type == UpdateKnowledge.DELETE:
            succ = self.knowledge_base.delete_action(pddl_action_dto)

        res.success = succ
        return res

    def delete_all_actions(self,
                           req: Empty.Request,
                           res: Empty.Response) -> Empty.Response:
        """ srv callback to delete all actions

        Args:
            req (Empty.Request): empty request
            res (Empty.Response): empty response

        Returns:
            Empty.Response: empty response
        """

        self.knowledge_base.delete_all_actions()

        return res


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2KnowledgeBaseNode()

    node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
