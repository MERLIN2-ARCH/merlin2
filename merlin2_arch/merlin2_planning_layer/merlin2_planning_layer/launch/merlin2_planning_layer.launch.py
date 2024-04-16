# Copyright (C) 2023  Miguel Ángel González Santamarta

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


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

    #
    # ARGS
    #
    dao_family = LaunchConfiguration("dao_family")
    dao_family_cmd = DeclareLaunchArgument(
        "dao_family",
        default_value=str(int(DaoFamilies.ROS2)),
        description="DAO family")

    mongo_uri = LaunchConfiguration("mongo_uri")
    mongo_uri_cmd = DeclareLaunchArgument(
        "mongo_uri",
        default_value="mongodb://localhost:27017/merlin2",
        description="MongoDB URI")

    planner = LaunchConfiguration("planner")
    planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value="1",
        description="PDDL planner")

    #
    # NODES
    #
    pddl_generator_node_cmd = Node(
        package="merlin2_pddl_generator",
        executable="pddl_generator_node",
        name="pddl_generator_node",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    planner_node_cmd = Node(
        package="merlin2_planner",
        executable="planner_node",
        name="planner_node",
        parameters=[{"planner": planner}]
    )

    plan_dispatcher_node_cmd = Node(
        package="merlin2_plan_dispatcher",
        executable="plan_dispatcher_node",
        name="plan_dispatcher_node",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    executor_node_cmd = Node(
        package="merlin2_executor",
        executable="executor_node",
        name="executor_node",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    knowledge_base_node_cmd = Node(
        package="kant_knowledge_base",
        executable="knowledge_base_node.py",
        name="knowledge_base_node",
        namespace="merlin2",
        condition=LaunchConfigurationEquals(
            "dao_family", str(int(DaoFamilies.ROS2)))
    )

    #
    # ADD
    #
    ld = LaunchDescription()

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)
    ld.add_action(planner_cmd)

    ld.add_action(pddl_generator_node_cmd)
    ld.add_action(planner_node_cmd)
    ld.add_action(plan_dispatcher_node_cmd)
    ld.add_action(executor_node_cmd)
    ld.add_action(knowledge_base_node_cmd)

    return ld
