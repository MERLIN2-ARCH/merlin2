#!/bin/bash

declare -a pkgs=("kant_dto" "kant_dao" "kant_knowledge_base" "yasmin" "yasmin_ros" "merlin2_planner" "merlin2_pddl_generator" "merlin2_plan_dispatcher")
 
for pkg in ${pkgs[@]}; do
    ros2 run $pkg run_pytests.sh
done
