
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
