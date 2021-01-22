
import rclpy

from merlin2_goal_dispatcher import Merlin2GoalDispatcher

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
    person_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
    person_at
)

from pddl_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from .pddl import person_attended

from custom_ros2 import Node


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
        miguel = PddlObjectDto(person_type, "miguel")
        objects = [kitchen, bedroom, livingroom, entrance, bathroom, miguel]

        robot_at_prop = PddlPropositionDto(robot_at, [entrance])
        miguel_at_prop = PddlPropositionDto(person_at, [miguel, livingroom])
        self.person_attended_goal = PddlPropositionDto(
            person_attended, [miguel], is_goal=True)
        propositions = [robot_at_prop, miguel_at_prop]

        pddl_object_dao.delete_all()
        pddl_proposition_dao.delete_all()

        for pddl_object_dto in objects:
            pddl_object_dao.save(pddl_object_dto)

        for pddl_proposition_dto in propositions:
            pddl_proposition_dao.save(pddl_proposition_dto)

    def execute(self):

        succeed = self.goal_dispatcher.execute_goals(
            [self.person_attended_goal])
        self.get_logger().info(str(succeed))


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2DemoNode()

    node.execute()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
