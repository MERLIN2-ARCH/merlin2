
""" Merlin2 Pddl Proposition Dao """

from typing import List
import asyncio
from rclpy.node import Node

from pddl_dao.pddl_dao_interface import PddlPropositionDao
from pddl_dto import PddlPropositionDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlProposition,
    GetPddlProposition
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser import (
    DtoMsgParser,
    MsgDtoParser
)


class Merlin2PddlPropositionDao(PddlPropositionDao):
    """ Merlin2 Pddl Proposition Dao Class """

    def __init__(self, node: Node):

        PddlPropositionDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlProposition, "get_propositions")

        self._update_client = self.node.create_client(
            UpdatePddlProposition, "update_proposition")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_propositions")

    async def _merlin2_get(self,
                           get_type: int,
                           predicate_name: str = "") -> List[PddlPropositionDto]:
        """ asyn merlin2_get method

        Args:
            predicate_name (str): predicate name

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        req = GetPddlProposition.Request()

        req.predicate_name = predicate_name
        req.get_type = get_type

        self._get_client.wait_for_service()
        future = self._get_client.call_async(req)

        try:
            await future
        except Exception as e:
            self.node.get_logger().info("Service call failed %r" % (e,))
            return None

        result = future.result()
        pddl_proposition_dto_list = []

        for pddl_proposition_msg in result.pddl_propositions:
            pddl_proposition_dto = self.msg_dto_parser.proposition_msg_to_dto(
                pddl_proposition_msg)
            pddl_proposition_dto_list.append(pddl_proposition_dto)

        return pddl_proposition_dto_list

    async def _merlin2_update(self,
                              pddl_proposition_dto: PddlPropositionDto,
                              update_type: int) -> bool:
        """ asyn merlin2_delete_all method

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to update

        Returns:
            boo: succeed
        """

        req = UpdatePddlProposition.Request()

        req.pddl_proposition = self.dto_msg_parser.proposition_dto_to_msg(
            pddl_proposition_dto)
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
        """ asyn delete_all method

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

    def get_by_predicate(self, predicate_name: str) -> List[PddlPropositionDto]:
        """ get a PddlPropositionDto list with a given predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPropositionDto: list of PddlPropositionDto of the pddl predicate name
        """

        pddl_proposition_dto_list = asyncio.run(
            self._merlin2_get(GetPddlProposition.Request.BY_PREDICATE,
                              predicate_name=predicate_name))

        return pddl_proposition_dto_list

    def get_goals(self) -> List[PddlPropositionDto]:
        """ get a PddlPropositionDto list of goals

        Returns:
            PddlPropositionDto: list of PddlPropositionDto goals
        """

        pddl_proposition_dto_list = asyncio.run(
            self._merlin2_get(GetPddlProposition.Request.GOALS))

        return pddl_proposition_dto_list

    def get_no_goals(self) -> List[PddlPropositionDto]:
        """ get a PddlPropositionDto list of no goals

        Returns:
            PddlPropositionDto: list of PddlPropositionDto no goals
        """

        pddl_proposition_dto_list = asyncio.run(
            self._merlin2_get(GetPddlProposition.Request.NO_GOALS))

        return pddl_proposition_dto_list

    def get_all(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of all PddlPropositionDto
        """
        pddl_proposition_dto_list = asyncio.run(
            self._merlin2_get(GetPddlProposition.Request.ALL))

        return pddl_proposition_dto_list

    def _save(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save a PddlPropositionDto
            if the PddlPropositionDto is already saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to save

        Returns:
            bool: succeed
        """

        predicate_name = pddl_proposition_dto.get_pddl_predicate().get_predicate_name()
        if not pddl_proposition_dto in self.get_by_predicate(predicate_name):
            succ = asyncio.run(self._merlin2_update(
                pddl_proposition_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def _update(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ update a PddlPropositionDto
            if the PddlPropositionDto is not saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to update

        Returns:
            bool: succeed
        """

        predicate_name = pddl_proposition_dto.get_pddl_predicate().get_predicate_name()
        if pddl_proposition_dto in self.get_by_predicate(predicate_name):
            succ = asyncio.run(self._merlin2_update(
                pddl_proposition_dto, UpdateKnowledge.SAVE))
            return succ

        return False

    def save(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save or update a PddlPropositionDto
            if the PddlPropositionDto is not saved it will be saved, else it will be updated

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to save or update

        Returns:
            bool: succeed
        """

        predicate_name = pddl_proposition_dto.get_pddl_predicate().get_predicate_name()
        if not pddl_proposition_dto in self.get_by_predicate(predicate_name):
            succ = self._save(pddl_proposition_dto)
        else:
            succ = self._update(pddl_proposition_dto)

        return succ

    def delete(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ delete a PddlPropositionDto
            if the PddlPropositionDto is not saved return False, else return True

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto to delete

        Returns:
            bool: succeed
        """

        succ = asyncio.run(self._merlin2_update(
            pddl_proposition_dto, UpdateKnowledge.DELETE))
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl propositions

        Returns:
            bool: succeed
        """

        asyncio.run(self._merlin2_delete_all())

        return True
