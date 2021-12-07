
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ament_index_python


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
    # NODES
    #

    merlin2_navigation_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_navigation_fsm_action",
        name="navigation"
    )

    merlin2_check_wp_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_check_wp_action",
        name="check_wp"
    )

    merlin2_mdpi_node_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_mdpi_node",
        name="merlin2_mdpi_node"
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
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer_launch.py"))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(topological_nav_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_check_wp_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_mdpi_node_cmd)

    return ld
