from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

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

    pddl_generator_node_cmd = Node(
        package="merlin2_pddl_generator",
        executable="pddl_generator_node",
        name="pddl_generator_node",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    planner_node_cmd = Node(
        package="merlin2_planner",
        executable="planner_node",
        name="planner_node",
    )

    plan_dispatcher_node_cmd = Node(
        package="merlin2_plan_dispatcher",
        executable="plan_dispatcher_node",
        name="plan_dispatcher_node",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    executor_node_cmd = Node(
        package="merlin2_executor",
        executable="executor_node",
        name="executor_node",
        parameters=[{"dao_family": dao_family,
                     "mongo_uri": mongo_uri}]
    )

    knowledge_base_node_cmd = Node(
        package="kant_knowledge_base",
        executable="knowledge_base_node.py",
        name="knowledge_base_node",
        namespace="merlin2",
        condition=LaunchConfigurationEquals(
            "dao_family", str(int(DaoFamilies.ROS2)))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)

    ld.add_action(pddl_generator_node_cmd)
    ld.add_action(planner_node_cmd)
    ld.add_action(plan_dispatcher_node_cmd)
    ld.add_action(executor_node_cmd)
    ld.add_action(knowledge_base_node_cmd)

    return ld
