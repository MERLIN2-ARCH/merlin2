#!/bin/bash

ros2 run kant_knowledge_base knowledge_base_node &
pid=$!

pkg=merlin2_pddl_generator
ros_pkg_path=$(python3 -c "from ament_index_python.packages import get_package_prefix; print(get_package_prefix('$pkg'))")
py_pkg_path=$(python3 -c "import os, $pkg; print(os.path.dirname($pkg.__file__))")
python3 -m pytest --cov=$py_pkg_path $ros_pkg_path/pytests/*.py

pkill -9 -e -f knowledge_base_node
wait $pid
