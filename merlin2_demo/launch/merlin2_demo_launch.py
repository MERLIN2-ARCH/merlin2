
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")
    topological_nav_share_dir = get_package_share_directory(
        "topological_nav")
    speech_to_text_share_dir = get_package_share_directory(
        "speech_to_text")
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

    #
    # NODES
    #

    merlin2_navigation_action_cmd = Node(
        package="merlin2_basic_actions",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    merlin2_hi_navigation_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_hi_navigation_fsm_action",
        name="hi_navigation",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    merlin2_demo_node_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_demo_node",
        name="merlin2_demo_node",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    #
    # LAUNCHES
    #

    topological_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(topological_nav_share_dir, "topological_nav_launch.py"))
    )

    speech_to_text_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_to_text_share_dir, "speech_to_text_launch.py"))
    )

    text_to_speech_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(text_to_speech_share_dir, "text_to_speech_launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer_launch.py")),
        launch_arguments={"dao_family": dao_family}.items()
    )

    ld = LaunchDescription()

    #
    # ADD
    #

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)

    ld.add_action(topological_nav_cmd)
    ld.add_action(speech_to_text_cmd)
    ld.add_action(text_to_speech_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_hi_navigation_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_demo_node_cmd)

    return ld
