# merlin2_pddl_generator

Shell 1
```shell
sudo service mongod start
ros2 run kant_knowledge_base knowledge_base_node
```

Shell 2
```shell
cd ~/ros2_ws/src/merlin2/merlin2_arch/merlin2_planning_layer/merlin2_pddl_generator/
pytest --cov=/home/miguel/ros2_ws/install/merlin2_pddl_generator/lib/python3.8/site-packages/merlin2_pddl_generator ./test/merlin2_pddl_generator_tests/*.py
```