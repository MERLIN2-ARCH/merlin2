
""" Merlin2 Pddl Dao Facory """

from rclpy.node import Node

from pddl_dao.merlin2_pddl_dao import (
    Merlin2PddlTypeDao,
    Merlin2PddlObjectDao,
    Merlin2PddlPredicateDao,
    Merlin2PddlPropositionDao,
    Merlin2PddlActionDao
)

from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class Merlin2PddlDaoFactory(PddlDaoFactory):
    """ Merlin2 Pddl Dao Facory Class """

    def __init__(self, node: Node):
        self.set_node(node)

    def get_node(self) -> Node:
        """ node getter

        Returns:
            None: node
        """

        return self._node

    def set_node(self, node: Node):
        """ node setter

        Args:
            node (None): Node
        """

        self._node = node

    def create_pddl_type_dao(self) -> Merlin2PddlTypeDao:
        """ create a merlin2 pddl dao type object

        Returns:
            Merlin2PddlTypeDao: merlin2 dao for pddl type
        """

        return Merlin2PddlTypeDao(self._node)

    def create_pddl_predicate_dao(self) -> Merlin2PddlPredicateDao:
        """ create a merlin2 pddl dao predicate object

        Returns:
            Merlin2PddlPredicateDao: merlin2 dao for pddl predicate
        """

        return Merlin2PddlPredicateDao(self._node)

    def create_pddl_action_dao(self) -> Merlin2PddlActionDao:
        """ create a merlin2 pddl dao action object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Merlin2PddlActionDao: merlin2 dao for pddl action
        """

        return Merlin2PddlActionDao(self._node)

    def create_pddl_object_dao(self) -> Merlin2PddlObjectDao:
        """ create a merlin2 pddl dao object object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Merlin2PddlObjectDao: merlin2 dao for pddl object
        """

        return Merlin2PddlObjectDao(self._node)

    def create_pddl_proposition_dao(self) -> Merlin2PddlPropositionDao:
        """ create a merlin2 pddl dao proposition object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            Merlin2PddlPropositionDao: merlin2 dao for pddl proposition
        """

        return Merlin2PddlPropositionDao(self._node)
