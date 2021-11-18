//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "moveit.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;
using namespace Eigen;
using namespace std::chrono_literals;

enum gripper_state
{
    opened = 35,
    closed = 0
};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group->execute(plan);
            //move_group->asyncExecute(plan);
            return true;
        }
    }
    throw "Couldn't plan a path";
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface *hand_move_group, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group);
}

bool goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::msg::Pose pose)
{
    move_group->setPoseTarget(pose);
    return wait_for_exec(move_group);
}

class GetPose : public rclcpp::Node
{
public:
    GetPose() : Node("get_pose")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/throwing_object/pose", 10, std::bind(&GetPose::listener_callback, this, std::placeholders::_1));

    };
    geometry_msgs::msg::Pose pose;

private:
    void listener_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        pose = *msg;
        pose.position.z += 0.09;

    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto pose_node = std::make_shared<GetPose>();

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(pose_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");

    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(10);

    bool use_spawn_obj;
    if (!move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        // In case the parameter was not created use default
        use_spawn_obj = false;
    }

    // Planning to a Pose goal
    if (use_spawn_obj)
    {
        auto pose = pose_node->pose;
        hand_move_group.setMaxVelocityScalingFactor(1.0);
        hand_move_group.setMaxAccelerationScalingFactor(1.0);
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group.getPlanningFrame();

        // The id of the object is used to identify it.
        collision_object.id = "box1";

        // Define a box to add to the world.
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.9;
        primitive.dimensions[1] = 0.9;
        primitive.dimensions[2] = 0.5;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.6;
        box_pose.position.y = 0;
        box_pose.position.z = -0.35;
        box_pose.position.z += primitive.dimensions[1] / 2;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);


        // Now, let's add the collision object into the world
        planning_scene_interface.addCollisionObjects(collision_objects);

        auto start_pose = move_group.getCurrentPose().pose;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");

        change_gripper(&hand_move_group, gripper_state::opened);

        pose = pose_node->pose;
        Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());

        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to object pose");
        pose.position.z += 0.1; // approach
        goto_pose(&move_group, pose);

        move_group.setMaxVelocityScalingFactor(0.1);
        move_group.setMaxAccelerationScalingFactor(0.1);
        hand_move_group.setMaxVelocityScalingFactor(0.1);
        hand_move_group.setMaxAccelerationScalingFactor(0.1);

        pose.position.z -= 0.09; // approach
        goto_pose(&move_group, pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing hand");
        change_gripper(&hand_move_group,  gripper_state::closed);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifting");
        pose.position.z += 0.1;
        goto_pose(&move_group,  pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to target pose");
        pose.position.x += 0.2;
        goto_pose(&move_group,  pose);

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering");
        // pose.position.z -= 0.1;
        // goto_pose(&move_group, server, pose);

        hand_move_group.setMaxVelocityScalingFactor(1.0);
        hand_move_group.setMaxAccelerationScalingFactor(1.0);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group,  gripper_state::opened);

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to start pose");
        goto_pose(&move_group,  start_pose);
        auto new_pose = pose_node->pose;

        if ((new_pose.position.x < pose.position.x - 0.05) || (pose.position.x + 0.05 < new_pose.position.x))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cube is not in bound");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
        }

    }
    rclcpp::shutdown();
    return 0;
}