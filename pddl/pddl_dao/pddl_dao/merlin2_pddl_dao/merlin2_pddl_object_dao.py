
""" Merlin2 Pddl Object Dao """

from typing import List
import asyncio
from rclpy import Node

from pddl_dao.pddl_dao_interface.pddl_object_dao import PddlObjectDao
from pddl_dto.pddl_object_dto import PddlObjectDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlObject,
    GetPddlObject
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser.dto_msg_parser import DtoMsgParser
from merlin2_knowledge_base.merlin2_knowledge_base_parser.msg_dto_parser import MsgDtoParser


class Merlin2PddlObjectDao(PddlObjectDao):
    """ Merlin2 Pddl Object Dao Class """

    def __init__(self, node: Node):

        PddlObjectDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlObject, "get_objects")

        self._update_client = self.node.create_client(
            UpdatePddlObject, "update_object")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_objects")

    async def _merlin2_get(self, object_name: str = "") -> List[PddlObjectDto]:
        """ asyn merlin2_get method

        Args:
            object_name (str): object name

        Returns:
            List[PddlObjectDto]: list of PddlObjectDto
        """

        req = GetPddlObject.Request()

        req.object_name = object_name

        future = self._get_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return None

        result = future.result()
        pddl_object_dto_list = []

        for pddl_object_msg in result.pddl_objects:
            pddl_object_dto = self.msg_dto_parser.object_msg_to_dto(
                pddl_object_msg)
            pddl_object_dto_list.append(pddl_object_dto)

        return pddl_object_dto_list

    async def _merlin2_update(self,
                              pddl_object_dto: PddlObjectDto,
                              update_type: int) -> bool:
        """ asyn merlin2_delete_all method

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to update

        Returns:
            boo: succeed
        """

        req = UpdatePddlObject.Request()

        req.pddl_object = self.dto_msg_parser.object_dto_to_msg(
            pddl_object_dto)
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

    def get(self, object_name: str) -> PddlObjectDto:
        """ get a PddlObjectDto with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlObjectDto: PddlObjectDto of the pddl object name
        """

        pddl_object_dto_list = asyncio.run(self._merlin2_get(object_name))

        if len(pddl_object_dto_list) == 1:
            return pddl_object_dto_list[0]

        return None

    def get_all(self) -> List[PddlObjectDto]:
        """ get all PddlObjectDto

        Returns:
            List[PddlObjectDto]: list of all PddlObjectDto
        """

        pddl_object_dto_list = asyncio.run(self._merlin2_get())

        return pddl_object_dto_list

    def _save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save a PddlObjectDto
            if the PddlObjectDto is already saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_object_dto.get_object_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_object_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def _update(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ update a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_object_dto.get_object_name()):
            succ = asyncio.run(self._merlin2_update(
                pddl_object_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save or update a PddlObjectDto
            if the PddlObjectDto is not saved it will be saved, else it will be updated

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_object_dto.get_object_name()):
            succ = self._save(pddl_object_dto)
        else:
            succ = self._update(pddl_object_dto)

        return succ

    def delete(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ delete a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to delete

        Returns:
            bool: succeed
        """

        succ = asyncio.run(self._merlin2_update(
            pddl_object_dto, UpdateKnowledge.DELETE))
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl objects

        Returns:
            bool: succeed
        """

        asyncio.run(self._merlin2_delete_all())

        return True
