# MERLIN 2 (MachinEd Ros pLanINg)

<p align="center">
  <img src="./images/logo.png" width="50%" />
</p>

## Table of Contents

1. [Features](#features)
2. [Installation](#installation)
3. [Knowledge Base](#knowledge-base)
4. [Demos](#demos)

## Features

![](./images/architecture.png)

## Installation

### SMTPlan+

```shell
$ sudo apt install libz3-dev
```

### MERLIN2

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules ssh://git@niebla.unileon.es:5022/mgonzs/merlin2.git

# check packages installation
# KANT, YASMIN, simple_node

$ cd ~/ros2_ws
$ colcon build
```

## Demos

This demo is tested with [ros2_rb1](https://github.com/mgonzs13/ros2_rb1) world. The RB1 robot will start driving to specific points in the world. Half of the goals are canceled randomly. Distance and time are saved in a CSV file.

```shell
$ ros2 launch rb1_gazebo granny.launch.py
$ ros2 launch merlin2_demo merlin2_demo_launch.py
```
