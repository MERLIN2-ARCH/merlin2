
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ament_index_python


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        'merlin2_planning_layer')
    topo_nav_share_dir = get_package_share_directory(
        'ros2_topological_nav')
    tts_share_dir = get_package_share_directory(
        'ros2_text_to_speech')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    #
    # NODES
    #

    merlin2_navigation_action_cmd = Node(
        package='merlin2_basic_actions',
        executable='merlin2_navigation_action',
        name='merlin2_navigation_action'
    )

    merlin2_check_wp_action_cmd = Node(
        package='merlin2_demo',
        executable='merlin2_check_wp_action',
        name='merlin2_check_wp_action'
    )

    merlin2_mdpi_node_cmd = Node(
        package='merlin2_demo',
        executable='merlin2_mdpi_node',
        name='merlin2_mdpi_node',
        namespace='merlin2'
    )

    #
    # LAUNCHES
    #

    topo_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(topo_nav_share_dir, 'ros2_topo_nav_launch.py')),
        launch_arguments={'points': ament_index_python.get_package_share_directory(
            "merlin2_demo") + '/params/granny.yaml'}.items()

    )

    tts_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tts_share_dir, 'ros2_text_to_speech_launch.py'))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, 'merlin2_planning_layer_launch.py'))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(topo_nav_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_check_wp_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_mdpi_node_cmd)

    return ld
