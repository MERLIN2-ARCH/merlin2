
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from kant_dao.pddl_dao_factory import PddlDaoFamilies


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")
    topo_nav_share_dir = get_package_share_directory(
        "ros2_topological_nav")
    speech_recognition_share_dir = get_package_share_directory(
        "ros2_speech_recognition")
    tts_share_dir = get_package_share_directory(
        "ros2_text_to_speech")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    #
    # ARGS
    #

    pddl_dao_family = LaunchConfiguration("pddl_dao_family")
    pddl_dao_family_cmd = DeclareLaunchArgument(
        "pddl_dao_family",
        default_value=str(int(PddlDaoFamilies.ROS2)),
        description="DAO family")

    mongoengine_uri = LaunchConfiguration("mongoengine_uri")
    mongoengine_uri_cmd = DeclareLaunchArgument(
        "mongoengine_uri",
        default_value="mongodb://localhost:27017/merlin2",
        description="MongoDB URI")

    #
    # NODES
    #

    merlin2_navigation_action_cmd = Node(
        package="merlin2_basic_actions",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{"pddl_dao_family": pddl_dao_family,
                     "mongoengine_uri": mongoengine_uri}]
    )

    merlin2_hi_navigation_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_hi_navigation_fsm_action",
        name="hi_navigation",
        parameters=[{"pddl_dao_family": pddl_dao_family,
                     "mongoengine_uri": mongoengine_uri}]
    )

    merlin2_demo_node_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_demo_node",
        name="merlin2_demo_node",
        parameters=[{"pddl_dao_family": pddl_dao_family,
                     "mongoengine_uri": mongoengine_uri}]
    )

    #
    # LAUNCHES
    #

    topo_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(topo_nav_share_dir, "ros2_topo_nav_launch.py"))
    )

    speech_recognition_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_recognition_share_dir, "ros2_speech_recognition_launch.py"))
    )

    tts_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tts_share_dir, "ros2_text_to_speech_launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer_launch.py")),
        launch_arguments={"pddl_dao_family": pddl_dao_family}.items()
    )

    ld = LaunchDescription()

    #
    # ADD
    #

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(pddl_dao_family_cmd)
    ld.add_action(mongoengine_uri_cmd)

    ld.add_action(topo_nav_cmd)
    ld.add_action(speech_recognition_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_hi_navigation_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_demo_node_cmd)

    return ld
