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

#ifndef SIM_ACTION_SERVER_H
#define SIM_ACTION_SERVER_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace sim_action_server
{

class ActionServer : public rclcpp::Node
{
public:

    ActionServer(std::string node_name = "trajectory_control", std::string action_node_name="/follow_joint_trajectory");

    bool execute_plan(trajectory_msgs::msg::JointTrajectory trajectory);

private:

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
    bool common_goal_accepted = false;
    rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
    int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
    void common_goal_response(
        std::shared_future<rclcpp_action::ClientGoalHandle
        <control_msgs::action::FollowJointTrajectory>::SharedPtr> future);

    void common_result_response(
        const rclcpp_action::ClientGoalHandle
        <control_msgs::action::FollowJointTrajectory>::WrappedResult & result);

    void common_feedback(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);
};
} // namespace sim_action_server

#endif // SIM_ACTION_SERVER_H
