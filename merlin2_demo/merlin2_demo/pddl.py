
from merlin2_basic_actions.merlin2_basic_types import (
    person_type,
    wp_type,
    object_type
)

from kant_dto import (
    PddlPredicateDto,
    PddlTypeDto
)

person_attended = PddlPredicateDto("person_attended", [person_type])
wp_checked = PddlPredicateDto("wp_checked", [wp_type])
door_type = PddlTypeDto("door")
door_at = PddlPredicateDto("door_at", [door_type, wp_type])
door_checked = PddlPredicateDto("door_checked", [door_type])
sound_type = PddlTypeDto("sound")
sound_listened = PddlPredicateDto("sound_listened", [sound_type])
