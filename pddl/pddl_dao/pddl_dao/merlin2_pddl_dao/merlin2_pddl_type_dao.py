
""" Merlin2 Pddl Type Dao """

from typing import List
from rclpy.node import Node

from pddl_dao.pddl_dao_interface import PddlTypeDao
from pddl_dto import PddlTypeDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlType,
    GetPddlType
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser import (
    DtoMsgParser,
    MsgDtoParser
)


class Merlin2PddlTypeDao(PddlTypeDao):
    """ Merlin2 Pddl Type Dao Class """

    def __init__(self, node: Node):

        PddlTypeDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlType, "get_types")

        self._update_client = self.node.create_client(
            UpdatePddlType, "update_type")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_types")

    def _merlin2_get(self, type_name: str = "") -> List[PddlTypeDto]:
        """ merlin2_get method

        Args:
            type_name (str): type name

        Returns:
            List[PddlTypeDto]: list of PddlTypeDto
        """

        req = GetPddlType.Request()

        req.type_name = type_name

        self._get_client.wait_for_service()
        result = self._get_client.call(req)

        pddl_type_dto_list = []

        for pddl_type_msg in result.pddl_types:
            pddl_type_dto = self.msg_dto_parser.type_msg_to_dto(pddl_type_msg)
            pddl_type_dto_list.append(pddl_type_dto)

        return pddl_type_dto_list

    def _merlin2_update(self,
                        pddl_type_dto: PddlTypeDto,
                        update_type: int) -> bool:
        """  merlin2_update method

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to update

        Returns:
            bool: succeed
        """

        req = UpdatePddlType.Request()

        req.pddl_type = self.dto_msg_parser.type_dto_to_msg(pddl_type_dto)
        req.update_konwledge.update_type = update_type

        self._update_client.wait_for_service()
        result = self._update_client.call(req)

        return result.success

    def _merlin2_delete_all(self):
        """  merlin2_delete_all method

        Returns:
            bool: succeed
        """

        req = Empty.Request()

        self._delete_all_client.wait_for_service()
        self._delete_all_client.call(req)

    def get(self, type_name: str) -> PddlTypeDto:
        """ get a PddlTypeDto with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlTypeDto: PddlTypeDto of the pddl type name
        """

        pddl_type_dto_list = self._merlin2_get(type_name)

        if len(pddl_type_dto_list) == 1:
            return pddl_type_dto_list[0]

        return None

    def get_all(self) -> List[PddlTypeDto]:
        """ get all PddlTypeDto

        Returns:
            List[PddlTypeDto]: list of all PddlTypeDto
        """

        pddl_type_dto_list = self._merlin2_get()

        return pddl_type_dto_list

    def _save(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save a PddlTypeDto
            if the PddlTypeDto is already saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_type_dto.get_type_name()):
            succ = self._merlin2_update(
                pddl_type_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def _update(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ update a PddlTypeDto
            if the PddlTypeDto is not saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_type_dto.get_type_name()):
            succ = self._merlin2_update(
                pddl_type_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def save(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save or update a PddlTypeDto
            if the PddlTypeDto is not saved it will be saved, else it will be updated

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_type_dto.get_type_name()):
            succ = self._save(pddl_type_dto)
        else:
            succ = self._update(pddl_type_dto)

        return succ

    def delete(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ delete a PddlTypeDto
            if the PddlTypeDto is not saved return False, else return True

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto to delete

        Returns:
            bool: succeed
        """

        succ = self._merlin2_update(
            pddl_type_dto, UpdateKnowledge.DELETE)
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl types

        Returns:
            bool: succeed
        """

        self._merlin2_delete_all()

        return True
