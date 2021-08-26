from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals

from kant_dao.pddl_dao_factory import PddlDaoFamilies


def generate_launch_description():
    namespace = "merlin2"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    #
    # ARGS
    #

    pddl_dao_family = LaunchConfiguration("pddl_dao_family")
    pddl_dao_family_cmd = DeclareLaunchArgument(
        "pddl_dao_family",
        default_value=str(int(PddlDaoFamilies.KANT)),
        description="DAO family")

    mongoengine_uri = LaunchConfiguration("mongoengine_uri")
    mongoengine_uri_cmd = DeclareLaunchArgument(
        "mongoengine_uri",
        default_value="mongodb://localhost:27017/merlin2",
        description="MongoDB URI")

    #
    # NODES
    #

    pddl_generator_node_cmd = Node(
        package="merlin2_pddl_generator",
        executable="pddl_generator_node",
        name="pddl_generator_node",
        namespace=namespace,
        parameters=[{"pddl_dao_family": pddl_dao_family,
                     "mongoengine_uri": mongoengine_uri}]
    )

    planner_node_cmd = Node(
        package="merlin2_planner",
        executable="planner_node",
        name="planner_node",
        namespace=namespace,
    )

    plan_dispatcher_node_cmd = Node(
        package="merlin2_plan_dispatcher",
        executable="plan_dispatcher_node",
        name="plan_dispatcher_node",
        namespace=namespace,
        parameters=[{"pddl_dao_family": pddl_dao_family,
                     "mongoengine_uri": mongoengine_uri}]
    )

    executor_node_cmd = Node(
        package="merlin2_executor",
        executable="executor_node",
        name="executor_node",
        namespace=namespace,
    )

    knowledge_base_node_cmd = Node(
        package="kant_knowledge_base",
        executable="knowledge_base_node",
        name="knowledge_base_node",
        namespace=namespace,
        condition=LaunchConfigurationEquals(
            "pddl_dao_family", str(int(PddlDaoFamilies.KANT)))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(pddl_dao_family_cmd)
    ld.add_action(mongoengine_uri_cmd)

    ld.add_action(pddl_generator_node_cmd)
    ld.add_action(planner_node_cmd)
    ld.add_action(plan_dispatcher_node_cmd)
    ld.add_action(executor_node_cmd)
    ld.add_action(knowledge_base_node_cmd)

    return ld
