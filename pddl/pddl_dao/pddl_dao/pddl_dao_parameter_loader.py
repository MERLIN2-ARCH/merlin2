
""" Parameter Loader to manage the factory creation using ROS2 parameters """

from rclpy.node import Node
from pddl_dao.pddl_dao_factory import (
    PddlDaoFactoryFactory,
    PddlDaoFamilies,
)
from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class PddlDaoParameterLoader:
    """ PDDL DAO Parameter Loader Class """

    def __init__(self, node: Node):
        # param names
        pddl_dao_family_param_name = "pddl_dao_family"
        mongoengine_uri_param_name = "mongoengine_uri"

        # declaring params
        node.declare_parameter(pddl_dao_family_param_name,
                               PddlDaoFamilies.MONGOENGINE)
        node.declare_parameter(mongoengine_uri_param_name,
                               "mongodb://localhost:27017/merlin2")

        # getting params
        pddl_dao_family = node.get_parameter(
            pddl_dao_family_param_name).get_parameter_value().integer_value
        mongoengine_uri = node.get_parameter(
            mongoengine_uri_param_name).get_parameter_value().string_value

        # creating pddl action dao
        pddl_dao_factory_factory = PddlDaoFactoryFactory()
        self.__pddl_dao_factory = pddl_dao_factory_factory.create_pddl_dao_factory(
            pddl_dao_family, uri=mongoengine_uri, node=node)

    def get_pddl_dao_factory(self) -> PddlDaoFactory:
        """ return the pddl dao factory created in the constructor

        Returns:
            PddlDaoFactory: pddl dao family
        """

        return self.__pddl_dao_factory
