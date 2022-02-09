
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
    topological_nav_share_dir = get_package_share_directory(
        "topological_nav")
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
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    merlin2_check_wp_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_check_wp_fsm_action",
        name="check_wp",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    merlin2_mdpi_node_cmd = Node(
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

    topological_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(topological_nav_share_dir, "topological_nav_launch.py")),
        launch_arguments={"points": ament_index_python.get_package_share_directory(
            "merlin2_demo") + "/params/granny.yaml"}.items()
    )

    tts_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(text_to_speech_share_dir, "text_to_speech_launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer_launch.py")),
        launch_arguments={"dao_family": dao_family}.items()
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)
    ld.add_action(total_points_cmd)
    ld.add_action(time_to_cancel_cmd)
    ld.add_action(number_of_tests_cmd)
    ld.add_action(results_path_cmd)
    ld.add_action(world_cmd)

    ld.add_action(topological_nav_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_check_wp_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_mdpi_node_cmd)

    return ld
