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


import unittest
from merlin2_planner.merlin2_planner_factory import Merlin2PlannerFactory
from merlin2_planner import Merlin2Planners


class TestMerlin2Planner(unittest.TestCase):
    def setUp(self):
        super().setUp()
        self.maxDiff = None

        self.planner_factory = Merlin2PlannerFactory()

        self.domain = "(define (domain merlin2)\n(:requirements :typing :negative-preconditions :durative-actions)\n(:types\n\trobot\n\twp\n)\n(:predicates\n\t(robot_at ?r0 - robot ?w1 - wp)\n)\n(:durative-action navigation\n\t:parameters ( ?r - robot ?s - wp ?d - wp)\n\t:duration (= ?duration 10)\n\t:condition (and\n\t\t(at start (robot_at ?r ?s))\n\t)\n\t:effect (and\n\t\t(at start (not (robot_at ?r ?s)))\n\t\t(at end (robot_at ?r ?d))\n\t)\n)\n)\n"
        self.problem = "(define (problem merlin2_prb)\n(:domain merlin2)\n(:objects\n\trb1 - robot\n\twp1 - wp\n\twp2 - wp\n)\n(:init\n\t(robot_at rb1 wp1)\n)\n(:goal\n\t(robot_at rb1 wp2)\n)\n)\n"

    def test_merlin2_planner(self):

        planner = self.planner_factory.create_planner(Merlin2Planners.POPF)
        planner.generate_plan(self.domain, self.problem)
        print(planner.get_str_plan())
        self.assertTrue(planner.has_solution())
        self.assertEqual("\
Number of literals: 2\n\
Constructing lookup tables:\n\
Post filtering unreachable actions: \n\
\x1b[01;34mNo analytic limits found, not considering limit effects of goal-only operators\x1b[00m\n\
All the ground actions in this problem are compression-safe\n\
Initial heuristic = 1.000\n\
;;;; Solution Found\n\
; States evaluated: 2\n\
; Cost: 10.000\n\
; Time 0.00\n\
0.000: (navigation rb1 wp1 wp2)  [10.000]\n\
", planner.get_str_plan())

        self.assertEqual(
            "navigation", planner.get_plan_actions()[0].action_name)
