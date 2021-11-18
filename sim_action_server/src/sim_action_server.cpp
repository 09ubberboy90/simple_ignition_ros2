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

#include "sim_action_server.hpp"

namespace sim_action_server
{
ActionServer::ActionServer(std::string node_name, std::string action_node_name)
    : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "Node %s %s created", this->get_name(), action_node_name.c_str());

    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        action_node_name);

    bool response = action_client->wait_for_action_server(std::chrono::seconds(5));
    if (!response)
        throw std::runtime_error("could not get action server");
    RCLCPP_INFO(this->get_logger(), "Created action server");
}

bool ActionServer::execute_plan(trajectory_msgs::msg::JointTrajectory trajectory)
{
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
    opt.goal_response_callback = [this](auto n) { this->common_goal_response(n); };
    opt.result_callback = [this](auto n) { this->common_result_response(n); };
    opt.feedback_callback = [this](auto n, auto m) { this->common_feedback(n, m); };

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory = trajectory;

    auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "send goal call failed :(");
        return false;
    }

    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
        goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by server");

    // Wait for the server to be done with the goal
    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(this->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "get result call failed :(");
        return false;
    }

    return true;
}
void ActionServer::common_goal_response(
    std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
{
    RCLCPP_DEBUG(
        this->get_logger(), "common_goal_response time: %f",
        rclcpp::Clock().now().seconds());
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        common_goal_accepted = false;
    }
    else
    {
        common_goal_accepted = true;
    }
}

void ActionServer::common_result_response(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result)
{
    printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_INFO(this->get_logger(), "Unknown result code");
        return;
    }
}

void ActionServer::common_feedback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    std::string tmp = "feedback->desired.positions :";
    for (auto &x : feedback->desired.positions)
    {
        tmp.append(std::to_string(x));
        tmp.append("\t");
    }
    RCLCPP_DEBUG(this->get_logger(), tmp);
    tmp = "feedback->desired.velocities :";
    for (auto &x : feedback->desired.velocities)
    {
        tmp.append(std::to_string(x));
        tmp.append("\t");
    }
    RCLCPP_DEBUG(this->get_logger(), tmp);
}
} // namespace sim_action_server
