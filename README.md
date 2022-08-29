# MERLIN 2 (MachinEd Ros pLanINg)

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

## Table of Contents

1. [Diagrams](#diagrams)
2. [Installation](#installation)
   - [Planners](#planners)
   - [MERLIN2](#merlin2)
3. [Demos](#demos)

## Diagrams

![](./docs/cerebrum.png)
![](./docs/architecture.png)

## Installation

### Planners

- POPF

- SMTPlan+

```shell
$ sudo apt install libz3-dev
```

- [unified-planning](https://github.com/aiplan4eu/unified-planning)

```shell
$ pip install --pre unified-planning[pyperplan,tamer]
```

- VHPOP

### MERLIN2

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules ssh://git@niebla.unileon.es:5022/mgonzs/merlin2.git
$ cd merlin2
$ pip3 install -r requirements.txt

# check packages installation
# KANT, YASMIN, simple_node,
# merlin2_reactive_layer pacakges

$ cd ~/ros2_ws
$ colcon build
```

## Demos

This demo is tested with [ros2_rb1](https://github.com/mgonzs13/ros2_rb1) world. The RB1 robot will start driving to specific points in the world. Half of the goals are canceled randomly. Distance and time are saved in a CSV file.

```shell
$ ros2 launch rb1_gazebo granny.launch.py
$ ros2 launch merlin2_demo merlin2_demo2.launch.py
```
