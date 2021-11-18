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

using namespace std::chrono_literals;
using std::placeholders::_1;

void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request> request, std::string arg)
{
    request->name = arg;
}
void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request> request, std::string arg) {}

void result_handler(std::shared_ptr<rclcpp::Node> node, std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    std::set<std::string> banned{"panda", "ground_plane"};
    for (auto name : result.get()->model_names)
    {
        if (banned.count(name) == 0)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), name);
            service_caller<gazebo_msgs::srv::GetEntityState>(node, "get_entity_state", poses, name);
        }
    }
}

void result_handler(std::shared_ptr<rclcpp::Node> node, std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    poses->poses.push_back(result.get()->state.pose);
}

class GetParam : public rclcpp::Node
{
public:
    GetParam() : Node("get_global_param")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>("stop_updating_obj", 10, std::bind(&GetParam::topic_callback, this, _1));
    };
    bool get_param()
    {
        return param;
    }

private:
    bool param = false;
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        param = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("moveit_collision", node_options);
    auto parameter_server = std::make_shared<GetParam>();
    auto service_node = rclcpp::Node::make_shared("service_handler");
    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");
    bool gazebo;
    if (!move_group_node->get_parameter("gazebo", gazebo))
    {
        // In case the parameter was not created use default
        gazebo = false;
    }

    bool thrower;
    if (!move_group_node->get_parameter("thrower", thrower))
    {
        // In case the parameter was not created use default
        thrower = false;
    }
    bool use_spawn_obj;
    if (!move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        // In case the parameter was not created use default
        use_spawn_obj = false;
    }

    if (use_spawn_obj)
    {

        while (true)
        {

            geometry_msgs::msg::PoseArray poses;
            std::vector<moveit_msgs::msg::CollisionObject> collision_object;
            service_caller<gazebo_msgs::srv::GetModelList>(service_node, "get_model_list", &poses);
            for (int i = 0; i < poses.poses.size(); i++)
            {
                moveit_msgs::msg::CollisionObject obj;
                auto pose = poses.poses[i];
                obj.header.frame_id = "world";
                obj.id = "Box" + i;
                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                obj.operation = obj.ADD;
                if (i == 0 && !thrower) // table
                {
                    primitive.dimensions[0] = 0.92;
                    if (gazebo)
                    {
                        primitive.dimensions[2] = 0.58;
                        primitive.dimensions[1] = 0.92;
                    }
                    else
                    {
                        primitive.dimensions[1] = 0.5;
                        primitive.dimensions[2] = 0.914;
                    }
                    pose.position.z += primitive.dimensions[1] / 2;
                    
                    
                }
                else // cube
                {
                    printf("%i\n", i);
                    primitive.dimensions[0] = 0.05;
                    primitive.dimensions[1] = 0.05;
                    primitive.dimensions[2] = 0.05;
                    // pose.position.z += primitive.dimensions[1] / 2;
                }
                if (i == 1)
                {
                    obj.id = "target";
                    if (parameter_server->get_param())
                    {
                        continue;
                    }
                }

                obj.primitives.push_back(primitive);
                obj.primitive_poses.push_back(pose);
                collision_object.push_back(obj);
            }
            planning_scene_interface.applyCollisionObjects(collision_object);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add an object into the world");
        }
    }

    rclcpp::shutdown();
    return 0;
}