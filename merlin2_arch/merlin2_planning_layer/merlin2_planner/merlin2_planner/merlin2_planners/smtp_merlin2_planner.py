
""" SMTPlan+ Merlin2 Planner """

from merlin2_planner.merlin2_planners.popf_merlin2_planner import PopfMerlin2Planner
import ament_index_python


class SmtpMerlin2Planner(PopfMerlin2Planner):
    """ SMTPlan+ Merlin2 Planner """

    def __init__(self):
        super().__init__()

        self.planner_path = ament_index_python.get_package_share_directory(
            "merlin2_planner") + "/planners/SMTPlan"

        self.planner_cmd = "timeout 60 " + self.planner_path + " {} {} -u 1000"

    def _parse_plan(self):
        """ parse the current plan from str to
            list of PlanAction and check if has solution
        """

        self._plan_actions = []
        self._has_solution = False

        if not "No plan found" in self._str_plan:
            self._has_solution = True

            pddl_action_list = self.get_lines_with_actions(self._str_plan)

            for action in pddl_action_list:
                plan_action = self.parse_action_str(action)
                self._plan_actions.append(plan_action)
