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


"""Unified Planning Merlin2 Planner"""

from unified_planning.shortcuts import OneshotPlanner
from unified_planning.engines.results import PlanGenerationResultStatus
from unified_planning.io.pddl_reader import PDDLReader

import tempfile
from merlin2_planner.merlin2_planners.merlin2_planner import Merlin2Planner
from merlin2_msgs.msg import PlanAction


class UpMerlin2Planner(Merlin2Planner):
    """UP Merlin2 Planner"""

    def __init__(self) -> None:
        super().__init__()

        self.reader = PDDLReader()
        self._up_plan = None

    def _generate_plan(self, domain: str, problem: str) -> None:
        """create a ppdl plan

        Args:
            domain (str): str of a pddl domain
            problem (str): str of a pddl problem
        """

        self._up_plan = None

        domain_file = tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8")
        problem_file = tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8")

        domain_file.write(domain.replace("\\n", "\n").replace("\\t", "\t"))
        problem_file.write(problem.replace("\\n", "\n").replace("\\t", "\t"))

        domain_file.seek(0)
        problem_file.seek(0)

        pddl_problem = self.reader.parse_problem(
            str(domain_file.name), str(problem_file.name)
        )

        with OneshotPlanner(problem_kind=pddl_problem.kind) as planner:
            try:

                result = planner.solve(pddl_problem)

                if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:

                    self._has_solution = True
                    self._str_plan = str(result.plan)
                    self._up_plan = result.plan
                else:
                    self._has_solution = False

            except:
                self._has_solution = False

        domain_file.close()
        problem_file.close()

    def _parse_plan(self) -> None:
        """parse the current plan from str to
        list of PlanAction and check if has solution
        """

        if self._has_solution:

            for ele in self._up_plan._actions:

                action = ele

                if isinstance(ele, tuple):
                    action = ele[1]

                plan_action = PlanAction()
                plan_action.action_name = action.action.name

                for o in action.actual_parameters:
                    plan_action.objects.append(str(o.constant_value()))

                self._plan_actions.append(plan_action)
