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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),            

        ))

    robot_description_config = load_file(
        'ur_configs', 'urdf/panda/panda_arm_hand.urdf')
    robot_description = {'robot_description': robot_description_config}
    state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ign_pub = Node(package='simple_arm_control',
                   executable='republisher',
                   output='screen',
                   )
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                     '-name', 'panda',
                     '-x', '0',
                     '-z', '0',
                     '-Y', '0',
                     '-file', "/home/ubb/Documents/PersonalProject/VrController/ur_configs/urdf/panda/panda_arm_hand.sdf"],
                 output='screen')
    bridge = Node(package='ros_ign_bridge',
             executable='parameter_bridge',
             name='parameter_bridge_throwing_object_pose',
             output='screen',
             arguments=[
                 '/model/throwing_object/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(pkg_share, 'worlds', 'panda_throw.sdf'), " -r"],
            description='Ignition Gazebo arguments'),
        gazebo,
        ign_pub,
        spawn,
        state,
        bridge
    ])
