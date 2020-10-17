
""" Popf Merlin2 Planner """

import tempfile
import subprocess
from merlin2_planner.merlin2_planners.merlin2_planner import Merlin2Planner
import ament_index_python


class PopfMerlin2Planner(Merlin2Planner):
    """ Popf Merlin2 Planner Abstract """

    def __init__(self):
        popf_path = ament_index_python.get_package_share_directory(
            "merlin2_planner") + "/planners/popf"

        self.popf_command = "timeout 60 " + popf_path  # + " -n "

    def plan(self, domain: str, problem: str) -> str:
        domain_file = tempfile.NamedTemporaryFile(mode="w+")
        problem_file = tempfile.NamedTemporaryFile(mode="w+")

        domain_file.write(domain)
        problem_file.write(problem)

        domain_file.seek(0)
        problem_file.seek(0)

        process = subprocess.Popen(
            self.popf_command + " " + domain_file.name + " " + problem_file.name, stdout=subprocess.PIPE)
        output, error = process.communicate()
        process.wait()

        plan = output

        domain_file.close()
        problem_file.close()

        return plan
