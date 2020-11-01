
""" Merlin2 Pddl Generator Node """

import rclpy
from rclpy.node import Node

from merlin2_plan_sys_interfaces.srv import GeneratePddl
from merlin2_pddl_generator.merlin2_pddl_generator_factory.merlin2_pddl_generator_factory import(
    Merlin2PddlGeneratorFactory
)
from pddl_dao.pddl_dao_factory.pddl_dao_families import PddlDaoFamilies


class Merlin2PddlGeneratorNode(Node):
    """ Merlin2 Pddl Generator Node Class """

    def __init__(self):

        super().__init__('merlin2_pddl_generator_mode')

        pddl_generator_factory = Merlin2PddlGeneratorFactory()

        # param names
        pddl_dao_family_param_name = "pddl_dao_family"
        mongoengine_uri_param_name = "mongoengine_uri"

        # declaring params
        self.declare_parameter(pddl_dao_family_param_name,
                               PddlDaoFamilies.MONGOENGINE)
        self.declare_parameter(mongoengine_uri_param_name, None)

        # getting params
        pddl_dao_family = self.get_parameter(
            pddl_dao_family_param_name).get_parameter_value().integer_value
        mongoengine_uri = self.get_parameter(
            mongoengine_uri_param_name).get_parameter_value().string_value

        # creating pddl generator
        self.pddl_generator = pddl_generator_factory.create_pddl_generator(
            pddl_dao_family, uri=mongoengine_uri, node=self)

        # service servers
        self.__start_server = self.create_service(
            GeneratePddl, 'generate_pddl', self.__generate_pddl_srv)

    def __generate_pddl_srv(self,
                            req: GeneratePddl.Request,
                            res: GeneratePddl.Response) -> GeneratePddl.Response:
        """ generate pddl srv callback

        Args:
            req (GeneratePddl.Request): request (empty msg)
            res (GeneratePddl.Response): response (domain and problem)

        Returns:
            GeneratePddl.Response:  response (domain and problem)
        """

        pddl_generated = self.pddl_generator.generate_pddl()

        res.domain = pddl_generated[0]
        res.problem = pddl_generated[1]

        return res


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2PddlGeneratorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
