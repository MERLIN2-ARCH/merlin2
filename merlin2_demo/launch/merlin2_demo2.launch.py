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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ament_index_python

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")
    waypoint_navigation_share_dir = get_package_share_directory(
        "waypoint_navigation")
    text_to_speech_share_dir = get_package_share_directory(
        "text_to_speech")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

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

    total_points = LaunchConfiguration("total_points")
    total_points_cmd = DeclareLaunchArgument(
        "total_points",
        default_value="6",
        description="Total points")

    time_to_cancel = LaunchConfiguration("time_to_cancel")
    time_to_cancel_cmd = DeclareLaunchArgument(
        "time_to_cancel",
        default_value="10",
        description="Time to cancel each mission in seconds")

    number_of_tests = LaunchConfiguration("number_of_tests")
    number_of_tests_cmd = DeclareLaunchArgument(
        "number_of_tests",
        default_value="5",
        description="Number of tests")

    results_path = LaunchConfiguration("results_path")
    results_path_cmd = DeclareLaunchArgument(
        "results_path",
        default_value="~/",
        description="Path to store the results")

    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value="granny",
        description="World used in tests")

    #
    # NODES
    #
    merlin2_navigation_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri,
        }]
    )

    merlin2_check_wp_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_check_wp_fsm_action",
        name="check_wp",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_demo2_node_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_demo2_node",
        name="merlin2_demo2_node",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri,
                     "total_points": total_points,
                     "time_to_cancel": time_to_cancel,
                     "number_of_tests": number_of_tests,
                     "results_path": results_path,
                     "world": world}]
    )

    #
    # LAUNCHES
    #
    waypoint_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_navigation_share_dir, "waypoint_navigation.launch.py")),
        launch_arguments={"wps": ament_index_python.get_package_share_directory(
            "merlin2_demo") + "/params/granny.yaml"}.items()
    )

    tts_cmd = IncludeLaunchDescription(
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

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)
    ld.add_action(planner_cmd)

    ld.add_action(total_points_cmd)
    ld.add_action(time_to_cancel_cmd)
    ld.add_action(number_of_tests_cmd)
    ld.add_action(results_path_cmd)
    ld.add_action(world_cmd)

    ld.add_action(waypoint_navigation_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_check_wp_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_demo2_node_cmd)

    return ld
