import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from launch.actions import ExecuteProcess

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_share = get_package_share_directory(pkg_name)

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),)

    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                     '-name', 'panda',
                     '-x', '0',
                     '-z', '0',
                     '-Y', '0',
                    #   '-topic', "robot_description"],
                    #  '-file', '/workspaces/Ignition/ubb/.gazebo/models/panda_ignition/model.sdf'],
                     '-file', os.path.join(pkg_share, "urdf", "panda_ign.sdf",)], # using well defined sdf as converting urdf causes problems with the physics engine
                 output='screen')

    robot_description_config = xacro.process_file(
        os.path.join(
            pkg_share,
            "urdf",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )    


    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 control load_controller --set-state start {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(
                pkg_share, 'worlds', 'empty.sdf')," -r"],
            description='Ignition Gazebo arguments'),
        ignition,
        spawn,
        robot_state_publisher,
    ] + load_controllers
    )
