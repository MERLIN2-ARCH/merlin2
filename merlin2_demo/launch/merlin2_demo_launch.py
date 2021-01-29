
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        'merlin2_planning_layer')
    topo_nav_share_dir = get_package_share_directory(
        'ros2_topological_nav')
    speech_recognition_share_dir = get_package_share_directory(
        'ros2_speech_recognition')
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

    merlin2_hi_navigation_action_cmd = Node(
        package='merlin2_demo',
        executable='merlin2_hi_navigation_action',
        name='merlin2_hi_navigation_action'
    )

    merlin2_demo_node_cmd = Node(
        package='merlin2_demo',
        executable='merlin2_demo_node',
        name='merlin2_demo_node'
    )

    #
    # LAUNCHES
    #

    topo_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(topo_nav_share_dir, 'ros2_topo_nav_launch.py'))
    )

    speech_recognition_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_recognition_share_dir, 'ros2_speech_recognition_launch.py'))
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
    ld.add_action(speech_recognition_cmd)
    ld.add_action(tts_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_hi_navigation_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_demo_node_cmd)

    return ld
