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


""" Base class to create MERLIN2 missions """

from abc import ABC, abstractmethod
from typing import List

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from simple_node import Node

from .merlin2_goal_dispatcher import Merlin2GoalDispatcher


class Merlin2MissionNode(Node, ABC):
    """ MERLIN2 Mission Node Class """

    def __init__(
        self,
        node_name: str,
        reset_problem: bool = True,
        run_mission: bool = True
    ) -> None:

        super().__init__(node_name, namespace="merlin2")

        self.goal_dispatcher = Merlin2GoalDispatcher(self)
        dao_factory = self.goal_dispatcher.get_dao_factory()

        self.pddl_object_dao = dao_factory.create_pddl_object_dao()
        self.pddl_proposition_dao = dao_factory.create_pddl_proposition_dao()

        if reset_problem:
            self.pddl_object_dao.delete_all()
            self.pddl_proposition_dao.delete_all()

        objects = self.create_objects()
        propositions = self.create_propositions()

        for pddl_object_dto in objects:
            succeed = self.pddl_object_dao.save(pddl_object_dto)
            if not succeed:
                raise Exception("Wrong Object: " +
                                str(pddl_object_dto))

        for pddl_proposition_dto in propositions:
            succeed = self.pddl_proposition_dao.save(pddl_proposition_dto)
            if not succeed:
                raise Exception("Wrong Proposition: " +
                                str(pddl_proposition_dto))

        if run_mission:
            self.execute_mission()

    def execute_goals(self, goals: List[PddlPropositionDto]) -> bool:
        """ execute list of goals

        Args:
            goals (List[PddlPropositionDto]): list of goals

        Returns:
            bool: succeed?
        """

        return self.goal_dispatcher.execute_goals(goals)

    def execute_goal(self, goal: PddlPropositionDto) -> bool:
        """ execute one goal

        Args:
            goal (PddlPropositionDto): goal

        Returns:
            bool: succeed?
        """

        return self.goal_dispatcher.execute_goals([goal])

    def cancel_goals(self) -> None:
        """ cancel executor goals """

        self.goal_dispatcher.cancel_goals()

    def __hash__(self):
        return Node.__hash__(self)

    @abstractmethod
    def execute_mission(self) -> None:
        """ execute mission """

    @abstractmethod
    def create_objects(self) -> List[PddlObjectDto]:
        """ create initial pddl objects

        Returns:
            List[PddlObjectDto]: list of pddl objets
        """

    @abstractmethod
    def create_propositions(self) -> List[PddlPropositionDto]:
        """ create initial pddl propositions

        Returns:
            List[PddlPropositionDto]: list of pddl propositions
        """
