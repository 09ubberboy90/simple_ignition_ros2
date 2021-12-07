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
        ),)

    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                     '-name', 'panda',
                     '-x', '0',
                     '-z', '0',
                     '-Y', '0',
                     #  '-topic', "robot_description"],
                     '-file', '/workspaces/Ignition/ubb/.gazebo/models/panda_ignition/model.sdf'],
                    #  '-file', os.path.join(pkg_share, "urdf", "panda_ign.sdf",)],
                 output='screen')

    bridge = Node(package='ros_ign_bridge',
                  executable='parameter_bridge',
                  name='parameter_bridge_throwing_object_pose',
                  output='screen',
                  arguments=[
                      '/model/throwing_object/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
                      '/world/empty/model/panda/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                      '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                      '/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
                      '/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float'
                  ],
                  remappings=[
                      ('/world/empty/model/panda/joint_state', 'joint_states'),
                  ],
                  )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(
                pkg_share, 'worlds', 'panda_place.sdf'), " -r"],
            description='Ignition Gazebo arguments'),
        gazebo,
        spawn,
        bridge
    ])
