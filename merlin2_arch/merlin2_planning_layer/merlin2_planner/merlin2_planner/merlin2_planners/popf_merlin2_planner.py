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


"""Popf Merlin2 Planner"""

import tempfile
import subprocess
from typing import List
from merlin2_planner.merlin2_planners.merlin2_planner import Merlin2Planner
from merlin2_msgs.msg import PlanAction
import ament_index_python


class PopfMerlin2Planner(Merlin2Planner):
    """Popf Merlin2 Planner"""

    def __init__(self) -> None:
        super().__init__()

        self.planner_path = (
            ament_index_python.get_package_share_directory("merlin2_planner")
            + "/planners/popf"
        )

        self.planner_cmd = "timeout 60 " + self.planner_path + " {} {}"

    def _generate_plan(self, domain: str, problem: str) -> None:
        """create a ppdl plan

        Args:
            domain (str): str of a pddl domain
            problem (str): str of a pddl problem
        """

        domain_file = tempfile.NamedTemporaryFile(mode="w+")
        problem_file = tempfile.NamedTemporaryFile(mode="w+")

        domain_file.write(domain)
        problem_file.write(problem)

        domain_file.seek(0)
        problem_file.seek(0)

        process = subprocess.Popen(
            self.planner_cmd.format(str(domain_file.name), str(problem_file.name)).split(
                " "
            ),
            stdout=subprocess.PIPE,
        )

        process.wait()
        plan = str(process.stdout.read().decode("utf-8"))

        domain_file.close()
        problem_file.close()

        self._str_plan = plan

    def _parse_plan(self) -> None:
        """parse the current plan from str to
        list of PlanAction and check if has solution
        """

        if "Solution Found" in self._str_plan:
            self._has_solution = True

            pddl_action_list = self.get_lines_with_actions(self._str_plan)

            for action in pddl_action_list:
                plan_action = self.parse_action_str(action)
                self._plan_actions.append(plan_action)

    def get_lines_with_actions(self, pddl_plan: str) -> List[str]:
        """get the lines with actions

        Args:
            pddl_plan (str): pddl plan string

        Returns:
            List[str]: list of action string
        """

        pddl_action_list = []

        for line in pddl_plan.split("\n"):
            if "(" in line and ")" in line and "[" in line and "]" in line:
                pddl_action_list.append(line)

        return pddl_action_list

    def parse_action_str(self, action_str: str) -> PlanAction:
        """parse an action string

        Args:
            action_str (str): string of an action

        Returns:
            PlanAction: plan action parsed
        """

        action = PlanAction()

        init_index = action_str.index("(") + 1
        end_index = action_str.index(")")

        action_str_cutted = action_str[init_index:end_index]

        action_str_splitted = action_str_cutted.split(" ")

        action.action_name = action_str_splitted[0]
        action.objects = action_str_splitted[1:]

        return action
