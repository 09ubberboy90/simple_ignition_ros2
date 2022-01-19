import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_share = get_package_share_directory(pkg_name)

    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("simple_arm"),
            "urdf",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "simple_arm", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )


    panda = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'panda.launch.py'),
        ),)

    run_move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'test.launch.py'),
        ),)

    collision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'collision.launch.py'),
        ),)

    moveit_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'moveit_controller.launch.py'),
        ),)

    prestart_moveit = Node(package='simple_arm_control',
                           executable='prestart_moveit',
                            output='screen',
                            parameters=[robot_description,
                                        robot_description_semantic,
                                        kinematics_yaml,
                                           ],)


    timer_2 = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([
        prestart_moveit,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=prestart_moveit,
                on_exit=[collision,
                         TimerAction(
                             period=15., # Wait for all of them to spawn
                             actions=[
                                moveit_controller
                             ])
                        ],
            )
        ),
    ])))

    timer_1 = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([  # hack since you can't have recursive timer
        run_move_group_node,
        TimerAction(
            period=10.,
            actions=[
                timer_2
            ])
    ])))

    return LaunchDescription([
        panda,
        TimerAction(
            period=5.,
            actions=[
                timer_1
            ]
        )
    ])
