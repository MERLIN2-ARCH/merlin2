
""" Merlin2 Pddl Generator Node """

import rclpy

from merlin2_arch_interfaces.srv import GeneratePddl
from merlin2_pddl_generator.merlin2_pddl_generator import Merlin2PddlGenerator

from kant_dao import PddlDaoParameterLoader

from simple_node import Node


class Merlin2PddlGeneratorNode(Node):
    """ Merlin2 Pddl Generator Node Class """

    def __init__(self):

        super().__init__("merlin2_pddl_generator_node")

        # loading parameters
        pddl_dao_parameter_loader = PddlDaoParameterLoader(self)
        pddl_dao_factory = pddl_dao_parameter_loader.get_pddl_dao_factory()
        self.pddl_generator = Merlin2PddlGenerator(pddl_dao_factory)

        # service servers
        self.__start_server = self.create_service(
            GeneratePddl, "generate_pddl", self.__generate_pddl_srv)

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

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
