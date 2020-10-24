
""" Merlin2 Pddl Action Dao """

from typing import List
import asyncio
from rclpy.node import Node

from pddl_dao.pddl_dao_interface.pddl_action_dao import PddlActionDao
from pddl_dto.pddl_action_dto import PddlActionDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlAction,
    GetPddlAction
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser.dto_msg_parser import DtoMsgParser
from merlin2_knowledge_base.merlin2_knowledge_base_parser.msg_dto_parser import MsgDtoParser


class Merlin2PddlActionDao(PddlActionDao):
    """ Merlin2 Pddl Action Dao Class """

    def __init__(self, node: Node):

        PddlActionDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlAction, "get_actions")

        self._update_client = self.node.create_client(
            UpdatePddlAction, "update_action")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_actions")

    async def _merlin2_get(self, action_name: str = "") -> List[PddlActionDto]:
        """ asyn merlin2_get method

        Args:
            action_name (str): action name

        Returns:
            List[PddlActionDto]: list of PddlActionDto
        """

        req = GetPddlAction.Request()

        req.action_name = action_name

        future = self._get_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return None

        result = future.result()
        pddl_action_dto_list = []

        for pddl_action_msg in result.pddl_actions:
            pddl_action_dto = self.msg_dto_parser.action_msg_to_dto(
                pddl_action_msg)
            pddl_action_dto_list.append(pddl_action_dto)

        return pddl_action_dto_list

    async def _merlin2_update(self,
                              pddl_action_dto: PddlActionDto,
                              update_type: int) -> bool:
        """ asyn merlin2_delete_all method

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to update

        Returns:
            boo: succeed
        """

        req = UpdatePddlAction.Request()

        req.pddl_action = self.dto_msg_parser.action_dto_to_msg(
            pddl_action_dto)
        req.update_konwledge.update_type = update_type

        future = self._update_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return False

        result = future.result()

        return result.success

    async def _merlin2_delete_all(self):
        """ asyn delete_all method

        Returns:
            boo: succeed
        """

        req = Empty.Request()

        future = self._delete_all_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))

    def get(self, action_name: str) -> PddlActionDto:
        """ get a PddlActionDto with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlActionDto: PddlActionDto of the pddl action name
        """

        pddl_action_dto_list = asyncio.run(self._merlin2_get(action_name))

        if len(pddl_action_dto_list) == 1:
            return pddl_action_dto_list[0]

        return None

    def get_all(self) -> List[PddlActionDto]:
        """ get all PddlActionDto

        Returns:
            List[PddlActionDto]: list of all PddlActionDto
        """

        pddl_action_dto_list = asyncio.run(self._merlin2_get())

        return pddl_action_dto_list

    def _save(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save a PddlActionDto
            if the PddlActionDto is already saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_action_dto.get_action_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_action_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def _update(self, pddl_action_dto: PddlActionDto) -> bool:
        """ update a PddlActionDto
            if the PddlActionDto is not saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_action_dto.get_action_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_action_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def save(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save or update a PddlActionDto
            if the PddlActionDto is not saved it will be saved, else it will be updated

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_action_dto.get_action_name()):
            succ = self._save(pddl_action_dto)
        else:
            succ = self._update(pddl_action_dto)

        return succ

    def delete(self, pddl_action_dto: PddlActionDto) -> bool:
        """ delete a PddlActionDto
            if the PddlActionDto is not saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to delete

        Returns:
            bool: succeed
        """

        succ = asyncio.run(self._merlin2_update(
            pddl_action_dto, UpdateKnowledge.DELETE))
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl actions

        Returns:
            bool: succeed
        """

        asyncio.run(self._merlin2_delete_all())

        return True
