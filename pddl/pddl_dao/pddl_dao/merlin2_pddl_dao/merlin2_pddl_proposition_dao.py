
""" Merlin2 Pddl Proposition Dao """

from typing import List
import asyncio
from rclpy import Node

from pddl_dao.pddl_dao_interface.pddl_proposition_dao import PddlPropositionDao
from pddl_dto.pddl_proposition_dto import PddlPropositionDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlProposition,
    GetPddlProposition
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser.dto_msg_parser import DtoMsgParser
from merlin2_knowledge_base.merlin2_knowledge_base_parser.msg_dto_parser import MsgDtoParser


class Merlin2PddlPropositionDao(PddlPropositionDao):
    """ Merlin2 Pddl Proposition Dao Class """

    def __init__(self, node: Node):

        PddlPropositionDao.__init__(self)

        self.node = node
