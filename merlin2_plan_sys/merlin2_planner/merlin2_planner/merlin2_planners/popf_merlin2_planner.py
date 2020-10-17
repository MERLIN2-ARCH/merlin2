
""" Popf Merlin2 Planner """

import tempfile
import subprocess
from merlin2_planner.merlin2_planners.merlin2_planner import Merlin2Planner
import ament_index_python


class PopfMerlin2Planner(Merlin2Planner):
    """ Popf Merlin2 Planner Abstract """

    def __init__(self):
        self.popf_path = ament_index_python.get_package_share_directory(
            "merlin2_planner") + "/planners/popf"

    def plan(self, domain: str, problem: str) -> str:
        domain_file = tempfile.NamedTemporaryFile(mode="w+")
        problem_file = tempfile.NamedTemporaryFile(mode="w+")

        domain_file.write(domain)
        problem_file.write(problem)

        domain_file.seek(0)
        problem_file.seek(0)

        process = subprocess.Popen([
            "timeout",
            "60",
            self.popf_path,
            "-n",
            str(domain_file.name),
            str(problem_file.name)],
            stdout=subprocess.PIPE)

        process.wait()
        plan = str(process.stdout.read().decode("utf-8"))

        domain_file.close()
        problem_file.close()

        return plan
