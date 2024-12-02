#include "waypoint_reconfigure/waypoint_reconfigure_node.hpp"

namespace waypoint_reconfigure
{

WaypointReconfigureNode::WaypointReconfigureNode(const rclcpp::NodeOptions& options) : WaypointReconfigureNode("", options) {}

WaypointReconfigureNode::WaypointReconfigureNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("waypoint_reconfigure_node", name_space, options),
global_inflation_radius(get_parameter("global_inflation_radius").as_double()),
global_cost_scaling_factor(get_parameter("global_cost_scaling_factor").as_double()),
local_inflation_radius(get_parameter("local_inflation_radius").as_double()),
local_cost_scaling_factor(get_parameter("local_cost_scaling_factor").as_double())
{
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "global_costmap/global_costmap");
    param_change_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "param_override", std::bind(&WaypointReconfigureNode::param_override, this, std::placeholders::_1, std::placeholders::_2));
}

bool WaypointReconfigureNode::param_override(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "param_override");

    param_client_->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", global_inflation_radius)});
    param_client_->set_parameters({rclcpp::Parameter("inflation_layer.cost_scaling_factor")});

    response->success = true;
    response->message = "success";
}

}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    auto node = std::make_shared<waypoint_reconfigure::WaypointReconfigureNode>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}