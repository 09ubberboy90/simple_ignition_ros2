import subprocess as s
import re
import rclpy
import os
import sys
import time
import functools

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetEntityState, GetModelList
from gazebo_msgs.msg import EntityState

from rclpy.node import Node

import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class SpawnerNode(Node):
    def __init__(self, args=None):
        super().__init__("SpawnerNode")

        self.model_list_pattern = re.compile(r"\-(.*)")
        self.entity_pattern = re.compile(r"\[(.*?)\]\n.*\[(.*?)\]")

        cube_path = "https://fuel.ignitionrobotics.org/1.0/09ubberboy90/models/Box 5cm"
        self.objs = []
        self.poses = {}
        self.listeners = {}
        
        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)

        
        self.spawn_obj(cube_path, "target", position=[0.5,0.0,0.025]) # target must be second to spawn

        self.spawn_obj(cube_path, "cube_1", position=[1.5,0.065,0.025])
        self.spawn_obj(cube_path, "cube_2", position=[1.5,0.0,0.025])
        self.spawn_obj(cube_path, "cube_3", position=[1.5,-0.065,0.025])
        self.spawn_obj(cube_path, "cube_4", position=[1.5,-0.0335,0.075])
        self.spawn_obj(cube_path, "cube_5", position=[1.5,0.0335,0.075])
        self.spawn_obj(cube_path, "cube_6", position=[1.5,0.0,0.135])

        self.create_param_bridge()
        self.create_listeners()

    def create_listeners(self):
        for obj in self.objs:
            self.listeners[obj] = self.create_subscription(
                Pose,
                f'/model/{obj}/pose',
                functools.partial(self.listener_callback, name=obj),
                10)

    def listener_callback(self, msg:Pose, name:str):
        self.poses[name] = msg

    def create_param_bridge(self):
        params = [f"/model/{obj}/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose" for obj in self.objs]
        command = f"ros2 run ros_ign_bridge parameter_bridge {' '.join(params)}"
        self.get_logger().info(command)
        self.command = s.Popen(command, shell=True, env=os.environ)

    def spawn_obj(self, path,name, position=[0, 0, 0], rotation = [0,0,0]):

        spawn_cmd = f"""ign service -s /world/panda_place/create \
--reqtype ignition.msgs.EntityFactory \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'sdf: '\
'"<?xml version=\\"1.0\\" ?>'\
'<sdf version=\\"1.6\\">'\
'<include>'\
'<pose> {position[0]} {position[1]} {position[2]} {rotation[0]} {rotation[1]} {rotation[2]}</pose>'\
'<uri>{path}</uri>'\
'<plugin filename=\\"ignition-gazebo-pose-publisher-system\\" name=\\"ignition::gazebo::systems::PosePublisher\\">'\
'<publish_nested_model_pose>true</publish_nested_model_pose>'\
'<publish_link_pose>false</publish_link_pose>'\
'<publish_collision_pose>false</publish_collision_pose>'\
'<publish_visual_pose>false</publish_visual_pose>'\
'<update_frequency>1</update_frequency>'\
'</plugin>'\
'</include>'\
'</sdf>" '\
'name: "{name}" '\
'allow_renaming: false'"""
        model_string = s.run(spawn_cmd, capture_output=True, shell=True)
        self.objs.append(name)
    
    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        response.model_names = self.objs
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        pose = self.poses.get(request.name)
        if pose is not None:
            success = True
        else:
            success = False
            pose = Pose()

        state = EntityState()
        state.name = request.name
        state.pose = pose
        response.state = state
        response.success = success
        return response


def main(args=None):
    rclpy.init(args=args)


    spawner = SpawnerNode(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(spawner, executor=executor)
    spawner.command.terminate()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
