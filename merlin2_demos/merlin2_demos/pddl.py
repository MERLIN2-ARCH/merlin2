# Copyright (C) 2023 Miguel Ángel González Santamarta

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


from merlin2_basic_actions.merlin2_basic_types import person_type, wp_type

from kant_dto import PddlPredicateDto, PddlTypeDto

person_attended = PddlPredicateDto("person_attended", [person_type])
wp_checked = PddlPredicateDto("wp_checked", [wp_type])
door_type = PddlTypeDto("door")
door_at = PddlPredicateDto("door_at", [door_type, wp_type])
door_checked = PddlPredicateDto("door_checked", [door_type])
sound_type = PddlTypeDto("sound")
sound_listened = PddlPredicateDto("sound_listened", [sound_type])
