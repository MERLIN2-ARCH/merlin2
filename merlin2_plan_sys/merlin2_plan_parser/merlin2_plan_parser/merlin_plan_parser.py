
""" Merlin2 Plan Parser """

from typing import List


class Merlin2PlanParser:
    """ Merlin2 Plan Parser Class """

    def __init__(self):
        pass

    def get_lines_with_actions(self, pddl_plan: str) -> List[str]:
        """ get the lines with actions

        Args:
            pddl_plan (str): pddl plan string

        Returns:
            List[str]: list of action string
        """

        pddl_action_list = []

        for line in pddl_plan.split("\n"):
            if("(" in line and
                ")" in line and
                "[" in line and
                    "]" in line):
                pddl_action_list.append(line)

        return pddl_action_list

    def parse_action_str(self, action_str: str) -> List[str]:
        """ parse an action string

        Args:
            action_str (str): string of an action

        Returns:
            List[str]: list with action name and params
        """

        init_index = action_str.index("(")
        end_index = action_str.index(")")

        action_str_cutted = action_str[init_index:end_index]

        return action_str_cutted.split(" ")

    def parse_plan(self, plan_str: str) -> List[List[str]]:
        """ parse plan string

        Args:
            plan_str (str): pddl plan string

        Returns:
            List[List[str]]: list of pddl action parsed
        """

        pddl_action_list = self.get_lines_with_actions(plan_str)

        parsed_action_list = []

        for action in pddl_action_list:
            parsed_action_list.append(self.parse_action_str(action))

        return parsed_action_list
