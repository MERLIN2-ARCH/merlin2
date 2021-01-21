
from merlin2_basic_actions.merlin2_basic_types import (
    person_type
)

from pddl_dto import (
    PddlPredicateDto
)

person_attended = PddlPredicateDto("person_attended", [person_type])
