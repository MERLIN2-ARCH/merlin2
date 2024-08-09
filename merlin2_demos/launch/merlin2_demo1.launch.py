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


import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")
    waypoint_navigation_share_dir = get_package_share_directory(
        "waypoint_navigation")
    speech_to_text_share_dir = get_package_share_directory(
        "speech_to_text")
    text_to_speech_share_dir = get_package_share_directory(
        "text_to_speech")

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
    merlin2_navigation_action_cmd = Node(
        package="merlin2_basic_actions",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_hi_navigation_action_cmd = Node(
        package="merlin2_demos",
        executable="merlin2_hi_navigation_fsm_action",
        name="hi_navigation",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_demo_node_cmd = Node(
        package="merlin2_demos",
        executable="merlin2_demo_node",
        name="merlin2_demo_node",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    #
    # LAUNCHES
    #
    waypoint_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_navigation_share_dir, "waypoint_navigation.launch.py"))
    )

    speech_to_text_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_to_text_share_dir, "speech_to_text.launch.py"))
    )

    text_to_speech_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(text_to_speech_share_dir, "text_to_speech.launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer.launch.py")),
        launch_arguments={"dao_family": dao_family,
                          "mongo_uri": mongo_uri,
                          "planner": planner}.items()
    )

    ld = LaunchDescription()

    #
    # ADD
    #
    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)
    ld.add_action(planner_cmd)

    ld.add_action(waypoint_navigation_cmd)
    ld.add_action(speech_to_text_cmd)
    ld.add_action(text_to_speech_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_hi_navigation_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_demo_node_cmd)

    return ld
