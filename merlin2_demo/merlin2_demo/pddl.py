
from merlin2_basic_actions.merlin2_basic_types import (
    person_type,
    wp_type
)

from kant_dto import (
    PddlPredicateDto
)

person_attended = PddlPredicateDto("person_attended", [person_type])
wp_checked = PddlPredicateDto("wp_checked", [wp_type])
