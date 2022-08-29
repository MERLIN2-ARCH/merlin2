
""" VHPOP Merlin2 Planner """

from merlin2_planner.merlin2_planners.popf_merlin2_planner import PopfMerlin2Planner
import ament_index_python


class VhpopMerlin2Planner(PopfMerlin2Planner):
    """ VHPOP Merlin2 Planner """

    def __init__(self):
        super().__init__()

        self.planner_path = ament_index_python.get_package_share_directory(
            "merlin2_planner") + "/planners/vhpop"

        self.planner_cmd = "timeout 60 " + self.planner_path + " {} {}"
