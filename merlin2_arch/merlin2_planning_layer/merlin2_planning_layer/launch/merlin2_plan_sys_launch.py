from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import ament_index_python


def generate_launch_description():
    namespace = 'merlin2'

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    #
    # NODES
    #

    pddl_generator_node_cmd = Node(
        package='merlin2_pddl_generator',
        executable='pddl_generator_node',
        name='pddl_generator_node',
        namespace=namespace,
    )

    planner_node_cmd = Node(
        package='merlin2_planner',
        executable='planner_node',
        name='planner_node',
        namespace=namespace,
    )

    plan_dispatcher_node_cmd = Node(
        package='merlin2_plan_dispatcher',
        executable='plan_dispatcher_node',
        name='plan_dispatcher_node',
        namespace=namespace,
    )

    executor_node_cmd = Node(
        package="merlin2_executor",
        executable='executor_node',
        name='executor_node',
        namespace=namespace,
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(pddl_generator_node_cmd)
    ld.add_action(planner_node_cmd)
    ld.add_action(plan_dispatcher_node_cmd)
    ld.add_action(executor_node_cmd)

    return ld
