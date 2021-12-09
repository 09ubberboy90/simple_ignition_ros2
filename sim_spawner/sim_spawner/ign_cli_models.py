import subprocess as s
import re
import rclpy
import os
import sys

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
        self.entity_pattern = re.compile(r"      \[(.*?)\]")

        table_path = "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cafe table"
        cube_path = "https://fuel.ignitionrobotics.org/1.0/09ubberboy90/models/Box 5cm"

        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)
        self.objs = {}
        self.spawn_obj(table_path, "table", offset=[0.6, 0, -0.5])
        # for i in range(-5,6):
        #     for j in range(-5,6):
        #         print(f"Spawned at {i*0.5} {j*0.5}")
        #         self.spawn_obj("worlds/Cube.wbo", position = [i, 0, j])
        for x in range(3, 6):
            for y in range(-3, 4):
                self.spawn_obj(cube_path, "cube", [x/10, y/10, 0.4])



    def spawn_obj(self, path,name, position=[0, 0, 0], offset=[0, 0, 0], rotation = [0,0,0]):
        out = []
        for i, j in zip(position, offset):
            out.append(i+j)

        spawn_cmd = f"""ign service -s /world/panda_place/create \
--reqtype ignition.msgs.EntityFactory \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'sdf: '\
'"<?xml version=\\"1.0\\" ?>'\
'<sdf version=\\"1.6\\">'\
'<include>'\
'<pose> {out[0]} {out[1]} {out[2]} {rotation[0]} {rotation[1]} {rotation[2]}</pose>'\
'<uri>{path}</uri>'\
'</include>'\
'</sdf>" '\
'name: "{name}" '\
'allow_renaming: true'"""
        model_string = s.run(spawn_cmd, capture_output=True, shell=True, check=True)
    
    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        model_string = s.run("ign model --list", capture_output=True, shell=True, check=True)
        out = self.model_list_pattern.findall(model_string.stdout.decode())
        if type(out) == s.CalledProcessError:
            #Handle errors
            response.success = False
            return response

        model_list = [s.strip() for s in out]
        self.get_logger().info(f"Got {len(model_list)} models")
        response.model_names = model_list
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        model_string = s.run(f"ign model -m {request.name} -p", capture_output=True, shell=True, check=True) 
        out = self.entity_pattern.findall(model_string.stdout.decode())
        if type(out) == s.CalledProcessError:
            #Handle errors
            response.success = False
            return response

        try:
            out[0].split('|')
        except IndexError:
            self.get_logger().info(str(model_string))
            response.success = False
            return response

        obj_pos = [float(x) for x in out[0].split("|")]  
        obj_rot = [float(x) for x in out[1].split("|")]  
        success = True
        state = EntityState()
        state.name = request.name
        pose = Pose()
        try:    
            pose.position = self.get_postion(obj_pos)
            pose.orientation = self.get_rotation(obj_rot)
        except: # object got deleted
            success = False
        finally:    
            state.pose = pose
            response.state = state
            response.success = success
        return response

    def get_postion(self, obj_pose):
        position = Point()
        position.x = obj_pose[0]
        position.y = obj_pose[1]
        position.z = obj_pose[2]
        return position

    def get_rotation(self, obj_rot):
        rotation = Quaternion()
        quat = get_quaternion_from_euler(*  obj_rot)
        rotation.x = float(quat[0])
        rotation.y = float(quat[1])
        rotation.z = float(quat[2])
        rotation.w = float(quat[3])
        return rotation

def main(args=None):
    rclpy.init(args=args)

    spawner = SpawnerNode(args=args)

    rclpy.spin(spawner)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
