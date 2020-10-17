
""" Merlin2 Pddl Generator Node """

import rclpy
from rclpy.node import Node

from merlin2_plan_sys_interfaces.srv._generate_pddl import GeneratePddl

from merlin2_pddl_generator.merlin2_pddl_generators.mongoengine_merlin2_pddl_generator import (
    MongoengineMerlin2PddlGenerator
)


class Merlin2PddlGeneratorNode(Node):
    """ Merlin2 Pddl Generator Node Class """

    def __init__(self):

        super().__init__('merlin2_pddl_generator_mode')

        self.pddl_generator = MongoengineMerlin2PddlGenerator()

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
