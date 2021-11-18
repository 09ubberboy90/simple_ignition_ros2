# Adapted from Xavier Weiss' code at: https://github.com/DEUCE1957/Automatic-Waste-Sorting

import moveit_msgs
import geometry_msgs
import gazebo_msgs
import std_msgs
from std_srvs.srv import Empty
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, GetModelState, GetWorldProperties
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Point, Quaternion
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2mat
import os
import sys
import shutil
from os import path
import glob
import rclpy
from rclpy.node import Node
import random
import re
import numpy as np


def define_pose(x, y, z, **kwargs):
    """Returns a Pose based on x,y,z and optionally o_x,o_y,o_z and o_w"""
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    pose_target.orientation.x = kwargs.get("o_x", 0.0)
    pose_target.orientation.y = kwargs.get("o_y", 1.0)
    pose_target.orientation.z = kwargs.get("o_z", 0.0)
    pose_target.orientation.w = kwargs.get("o_w", 0.0)
    return pose_target


def get_rpy_quaternion(roll, pitch, yaw):
    """Converts human readable roll (about x-axis), pitch (about y-axis)
    and yaw (about z-axis) format to a 4D quaternion"""
    return mat2quat(euler2mat(roll, pitch, yaw, 'sxyz'))


def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
    """Returns whether 2 floats are equal with a relative and absolute 
    tolerance to prevent problems from floating point precision"""
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class BoundingBox(object):
    """Defines a cuboid volume to represent an object's bounding volume
    If only x is supplied, constructs a x*x*x cube
    If y is also supplied, constructs a x*y*x cuboid
    If y and z are supplied, constructs a x*y*z cuboid"""

    def __init__(self, x, y=None, z=None):  # x,y,z
        self.x = x
        self.y = x if y is None else y
        self.z = x if z is None and y is None else (y if z is None else z)


class ServiceNode(Node):
    def __init__(self, srv_type, srv_name):
        super().__init__("service_caller")
        self.cli = self.create_client(srv_type, srv_name)

        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

    def send_spawn(self, model_name, model_xml, robot_namespace, initial_pose, reference_frame):
        self.req = SpawnEntity.Request()
        self.req.name = model_name
        self.req.xml = model_xml
        self.req.robot_namespace = robot_namespace
        self.req.initial_pose = initial_pose
        self.req.reference_frame = reference_frame

        self.future = self.cli.call_async(self.req)

    def send_delete(self, model_name):
        self.req = DeleteEntity.Request()
        self.req.name = model_name

        self.future = self.cli.call_async(self.req)
    
    def send_pose(self, pose):
        self.future = self.cli.call_async(pose)


class ObjectSpawner(object):
    """Provides convenient API for spawning any objects in the Gazebo model directory

    Attributes:
        -- Provided at initalization --
        reference_frame: coordinates are used relative to ref frame [default: 'world']
        model_name: if not provided, interactively choose model [default: None]
        model_directory: if not provided, use default directory [default: None]
        verbose: How much daignostic info to print [default: 1]
    """

    def __init__(self, reference_frame="world",
                 model_name="obj_spawn", model_directory=None,
                 verbose=1, service=None):

        self.instances = {}
        self.reference_frame = reference_frame
        self.database = False
        if model_directory is None:
            self.model_dir = """\
                            <sdf version="1.6">
                                <world name="default">
                                    <include>
                                        <uri>model://{}</uri>
                                    </include>
                                </world>
                            </sdf>""".format(model_name)
            self.database = True
        else:
            self.model_dir = model_directory

        if model_name is None:
            self.model_name = self._choose_model()
        else:
            self.model_name = model_name


        self.verbose = verbose

    def call_service(self, srv_type, srv_name, model_name, model_xml=None, robot_namespace=None, initial_pose=None, reference_frame=None):

        service_node = ServiceNode(srv_type,srv_name)
        if srv_type == SpawnEntity:
            service_node.send_spawn(model_name, model_xml,robot_namespace,initial_pose,reference_frame)
        else:
            service_node.send_delete(model_name)

        rclpy.spin_once(service_node)
        if service_node.future.done():
            try:
                response = service_node.future.result()
            except Exception as e:
                service_node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                service_node.get_logger().info("Successfully executed service")
                service_node.get_logger().info(
                    'Result of service: '+response.status_message)

        service_node.destroy_node()


    def spawn_model(self, pose):
        """Spawns a model at the target pose (position and orientation).

        If Spawning service failed: Returns None
        Else: Returns name of generated object and its real pose"""
        if self.database:
            xml = self.model_dir
        else:     
            model_path = path.join(self.model_dir, self.model_name, "model.sdf")
            with open(model_path, "r") as f:
                xml = f.read()

        if self.verbose >= 3:
            print(">> Loaded Model {} <<<\n{}".format(self.model_name, xml))

        # Enforce unique names
        if len(self.instances) == 0:
            name = self.model_name
        else:
            name = "{}_{}".format(self.model_name, len(self.instances))
        self.call_service(SpawnEntity, '/spawn_entity', name, xml.replace("\n", ""),
                          "/", pose, self.reference_frame)

        self.instances[name] = pose

        return name, pose

    def _choose_model(self):
        """Interactively select model by traversing Gazebo model subdirectories
        Copies chosen model to top of Gazebo Model path so that it is accessible
        Returns the chosen model name"""
        # Only descend into N levels of file path tree
        def walklevel(some_dir, level=1):
            some_dir = some_dir.rstrip(os.path.sep)
            assert os.path.isdir(some_dir)
            num_sep = some_dir.count(os.path.sep)
            for root, dirs, files in os.walk(some_dir):
                yield root, dirs, files
                num_sep_this = root.count(os.path.sep)
                if num_sep + level <= num_sep_this:
                    del dirs[:]

        # Get all VALID folders with mesh options
        def get_listdir(path, verbose=True):
            valid_dirs = {}
            count = 0
            for dir_name, child_dirs, files in walklevel(path, level=1):
                if child_dirs in [[], ['meshes'], ['materials', 'meshes']]:
                    continue
                else:
                    if verbose:
                        print("{}: {}".format(count, dir_name))
                    valid_dirs[count] = dir_name
                    count += 1
            return valid_dirs

    def spawn_on_table(self, spawn=True, random_face=True,
                       table_bbox=BoundingBox(0.913, 0.913, 0.82),
                       table_offset=Point(x=-0.45, y=-0.45, z=-0.25)):
        """Spawns an object on a table's surface with random location and yaw. 

        Attributes:
            spawn: Whether to actually spawn the object [default:True]
            random_face: Whether to vary which face is front-facing (24 possibilities) [default: True]
            table_bbox: Defines table's bounding box, ensure this is reachable [default: 0.32*0.5*0.7825]
            table_offset: Where table is placed relative to world origin (0,0,0)
        Returns:
            Unique name in simulation, overhead RPY and real pose
        """
        x = random.uniform(table_offset.x, table_offset.x +
                           table_bbox.x)  # Default: 0.38->0.7
        y = random.uniform(table_offset.y, table_offset.y +
                           table_bbox.y)  # Default: -0.1->0.4)
        z = table_offset.z + table_bbox.z  # Default: 0.7825
        fixed_rotations = np.arange(0, 2*np.pi, np.pi/2)
        if random_face:
            roll, pitch, yaw = random.choice(fixed_rotations), \
                random.choice(fixed_rotations), \
                random.uniform(0, 2*np.pi)
        else:
            roll, pitch, yaw = 0, 0, 0

        q = get_rpy_quaternion(roll, pitch, yaw)

        pose = define_pose(x=x, y=y, z=z, o_x=q[0],
                           o_y=q[1], o_z=q[2], o_w=q[3])

        # ToDo: Use bounding boxes to avoid overlap

        name = None
        if spawn:
            name, _ = self.spawn_model(pose)

            if self.verbose >= 2:
                print("Spawned {} at (x={},y={},z={})".format(self.model_name,
                                                              x, y, z))

        return name if name is not None else "EMPTY", (np.pi, 0, yaw), pose

    def delete_models(self, instance_name=None):
        """Use class-wide instances attribute to delete models. 
        instance_name: if specified, deletes specific model [default: None]"""
        if instance_name is None:
            for instance_name in self.instances.keys():
                self.call_service(
                    DeleteEntity, '/delete_entity', instance_name)
                del self.instances[instance_name]
        elif instance_name in self.instances:
            self.call_service(
                DeleteEntity, '/delete_entity', instance_name)
            del self.instances[instance_name]


def main(args=None):
    rclpy.init()
    table_spawner = ObjectSpawner(reference_frame="world",
                                  model_name="cafe_table")
    table_spawner.spawn_model(Pose(position=Point(x=0.6, y=0.0, z=-0.35)))
    block_spawner = ObjectSpawner(reference_frame="world",
                                model_name="wood_cube_5cm")
    block_spawner.spawn_model(Pose(position=Point(x=0.2, y=0.0, z=0.65)))
    

    rclpy.shutdown()

if __name__ == '__main__':
    main()
