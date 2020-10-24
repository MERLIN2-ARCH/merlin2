
""" Merlin2 Pddl Action Dao """

from typing import List
import asyncio
from rclpy import Node

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
