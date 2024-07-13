#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" Merlin2 Pddl Generator Node """

import time
import rclpy

from merlin2_msgs.srv import GeneratePddl
from merlin2_pddl_generator.merlin2_pddl_generator import Merlin2PddlGenerator

from simple_node import Node
from kant_dao import ParameterLoader


class Merlin2PddlGeneratorNode(Node):
    """ Merlin2 Pddl Generator Node Class """

    def __init__(self):

        super().__init__("pddl_generator_node", namespace="merlin2")

        # loading parameters
        parameter_loader = ParameterLoader(self)
        dao_factory = parameter_loader.get_dao_factory()
        self.pddl_generator = Merlin2PddlGenerator(dao_factory)

        # service servers
        self.__generate_pddl_service = self.create_service(
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

        start_time = time.time()
        pddl_generated = self.pddl_generator.generate_pddl()
        end_time = time.time()

        elapsed_time = end_time - start_time
        self.get_logger().info(
            f"Time to generate PDDL: {elapsed_time} seconds")

        res.domain = pddl_generated[0]
        res.problem = pddl_generated[1]

        return res


def main():
    rclpy.init()

    node = Merlin2PddlGeneratorNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
