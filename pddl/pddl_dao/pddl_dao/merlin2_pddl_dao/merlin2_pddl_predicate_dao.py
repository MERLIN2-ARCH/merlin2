
""" Merlin2 Pddl Predicate Dao """

from typing import List
import asyncio
from rclpy import Node

from pddl_dao.pddl_dao_interface.pddl_predicate_dao import PddlPredicateDao
from pddl_dto.pddl_predicate_dto import PddlPredicateDto

from merlin2_knowledge_base_interfaces.srv import (
    UpdatePddlPredicate,
    GetPddlPredicate
)
from merlin2_knowledge_base_interfaces.msg import UpdateKnowledge
from std_srvs.srv import Empty

from merlin2_knowledge_base.merlin2_knowledge_base_parser.dto_msg_parser import DtoMsgParser
from merlin2_knowledge_base.merlin2_knowledge_base_parser.msg_dto_parser import MsgDtoParser


class Merlin2PddlPredicateDao(PddlPredicateDao):
    """ Merlin2 Pddl Predicate Dao Class """

    def __init__(self, node: Node):

        PddlPredicateDao.__init__(self)

        self.node = node
