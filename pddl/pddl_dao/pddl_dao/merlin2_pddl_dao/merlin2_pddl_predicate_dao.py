
""" Merlin2 Pddl Predicate Dao """

from typing import List
import asyncio
from rclpy.node import Node

from pddl_dao.pddl_dao_interface import PddlPredicateDao
from pddl_dto import PddlPredicateDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlPredicate,
    GetPddlPredicate
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser import (
    DtoMsgParser,
    MsgDtoParser
)


class Merlin2PddlPredicateDao(PddlPredicateDao):
    """ Merlin2 Pddl Predicate Dao Class """

    def __init__(self, node: Node):

        PddlPredicateDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlPredicate, "get_predicates")

        self._update_client = self.node.create_client(
            UpdatePddlPredicate, "update_predicate")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_predicates")

    async def _merlin2_get(self, predicate_name: str = "") -> List[PddlPredicateDto]:
        """ asyn merlin2_get method

        Args:
            predicate_name (str): predicate name

        Returns:
            List[PddlPredicateDto]: list of PddlPredicateDto
        """

        req = GetPddlPredicate.Request()

        req.predicate_name = predicate_name

        self._get_client.wait_for_service()
        future = self._get_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return None

        result = future.result()
        pddl_predicate_dto_list = []

        for pddl_predicate_msg in result.pddl_predicates:
            pddl_predicate_dto = self.msg_dto_parser.predicate_msg_to_dto(
                pddl_predicate_msg)
            pddl_predicate_dto_list.append(pddl_predicate_dto)

        return pddl_predicate_dto_list

    async def _merlin2_update(self,
                              pddl_predicate_dto: PddlPredicateDto,
                              update_type: int) -> bool:
        """ asyn merlin2_update method

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

        Returns:
            boo: succeed
        """

        req = UpdatePddlPredicate.Request()

        req.pddl_predicate = self.dto_msg_parser.predicate_dto_to_msg(
            pddl_predicate_dto)
        req.update_konwledge.update_type = update_type

        self._update_client.wait_for_service()
        future = self._update_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return False

        result = future.result()

        return result.success

    async def _merlin2_delete_all(self):
        """ asyn merlin2_delete_all method

        Returns:
            boo: succeed
        """

        req = Empty.Request()

        self._delete_all_client.wait_for_service()
        future = self._delete_all_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))

    def get(self, predicate_name: str) -> PddlPredicateDto:
        """ get a PddlPredicateDto with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto of the pddl predicate name
        """

        pddl_predicate_dto_list = asyncio.run(
            self._merlin2_get(predicate_name))

        if len(pddl_predicate_dto_list) == 1:
            return pddl_predicate_dto_list[0]

        return None

    def get_all(self) -> List[PddlPredicateDto]:
        """ get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of all PddlPredicateDto
        """

        pddl_predicate_dto_list = asyncio.run(self._merlin2_get())

        return pddl_predicate_dto_list

    def _save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save a PddlPredicateDto
            if the PddlPredicateDto is already saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_predicate_dto.get_predicate_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_predicate_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def _update(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ update a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_predicate_dto.get_predicate_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_predicate_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save or update a PddlPredicateDto
            if the PddlPredicateDto is not saved it will be saved, else it will be updated

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_predicate_dto.get_predicate_name()):
            succ = self._save(pddl_predicate_dto)
        else:
            succ = self._update(pddl_predicate_dto)

        return succ

    def delete(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ delete a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to delete

        Returns:
            bool: succeed
        """

        succ = asyncio.run(self._merlin2_update(
            pddl_predicate_dto, UpdateKnowledge.DELETE))
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl predicates

        Returns:
            bool: succeed
        """

        asyncio.run(self._merlin2_delete_all())

        return True
