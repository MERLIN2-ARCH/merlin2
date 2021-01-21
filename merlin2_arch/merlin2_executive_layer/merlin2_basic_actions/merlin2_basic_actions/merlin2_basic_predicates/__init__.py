
from pddl_dto import (
    PddlPredicateDto,
)
from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
    person_type
)

robot_at = PddlPredicateDto("robot_at", [wp_type])
person_at = PddlPredicateDto("person_at", [person_type, wp_type])
