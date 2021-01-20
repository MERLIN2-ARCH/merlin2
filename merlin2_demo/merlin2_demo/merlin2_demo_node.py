
import rclpy

from merlin2_goal_dispatcher import Merlin2GoalDispatcher
from custom_ros2 import Node
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from pddl_dto import (
    PddlObjectDto,
    PddlPropositionDto
)


class Merlin2DemoNode(Node):

    def __init__(self):

        super().__init__("mission_layer_node")

        self.goal_dispatcher = Merlin2GoalDispatcher(self)
        self.pddl_dao_factory = self.goal_dispatcher.get_pddl_factory()

        pddl_object_dao = self.pddl_dao_factory.create_pddl_object_dao()
        pddl_proposition_dao = self.pddl_dao_factory.create_pddl_proposition_dao()

        kitchen = PddlObjectDto(wp_type, "kitchen")
        bedroom = PddlObjectDto(wp_type, "bedroom")
        livingroom = PddlObjectDto(wp_type, "livingroom")
        entrance = PddlObjectDto(wp_type, "entrance")
        bathroom = PddlObjectDto(wp_type, "bathroom")
        objects = [kitchen, bedroom, livingroom, entrance, bathroom]

        robot_at_prop = PddlPropositionDto(robot_at, [entrance])
        self.robot_at_goal = PddlPropositionDto(
            robot_at, [kitchen], is_goal=True)

        pddl_object_dao.delete_all()
        pddl_proposition_dao.delete_all()

        for pddl_object_dto in objects:
            pddl_object_dao.save(pddl_object_dto)

        pddl_proposition_dao.save(robot_at_prop)

    def execute(self):

        succeed = self.goal_dispatcher.execute_goals([self.robot_at_goal])
        self.get_logger().info(str(succeed))


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2DemoNode()

    node.execute()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
