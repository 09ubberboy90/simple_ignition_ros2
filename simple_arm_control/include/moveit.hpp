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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sim_action_server.hpp"

#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"


void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request>, std::string);
void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request>, std::string);

void result_handler(std::shared_ptr<rclcpp::Node>,std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>>, geometry_msgs::msg::PoseArray *);
void result_handler(std::shared_ptr<rclcpp::Node>,std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>>, geometry_msgs::msg::PoseArray *);



template <typename MessageType>
void service_caller(std::shared_ptr<rclcpp::Node>, std::string, geometry_msgs::msg::PoseArray *, std::string arg="");

template <typename MessageType>
void service_caller(std::shared_ptr<rclcpp::Node> node, std::string srv_name, geometry_msgs::msg::PoseArray *poses, std::string arg)
{
    typename rclcpp::Client<MessageType>::SharedPtr model_list_cl =
        node->create_client<MessageType>(srv_name);

    auto request = std::make_shared<typename MessageType::Request>();
    if (!arg.empty())
    {
        namer(request, arg);
    }
    std::chrono::seconds timeout(1);
    while (!model_list_cl->wait_for_service(timeout))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = model_list_cl->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        result_handler(node, result, poses);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
}

