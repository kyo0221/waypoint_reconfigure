#include "waypoint_reconfigure/waypoint_reconfigure_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace waypoint_reconfigure
{

WaypointReconfigureNode::WaypointReconfigureNode(const rclcpp::NodeOptions& options) : WaypointReconfigureNode("", options) {}

WaypointReconfigureNode::WaypointReconfigureNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("waypoint_reconfigure_node", name_space, options)
{
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "global_costmap/global_costmap");
    param_override_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "param_override", std::bind(&WaypointReconfigureNode::param_override, this, std::placeholders::_1, std::placeholders::_2));

    param_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "param_reset", std::bind(&WaypointReconfigureNode::param_reset, this, std::placeholders::_1, std::placeholders::_2));
}

void WaypointReconfigureNode::param_override(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "params_override");

    try{
        param_client_->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", global_inflation_radius)});
        param_client_->set_parameters({rclcpp::Parameter("inflation_layer.cost_scaling_factor", global_cost_scaling_factor)});

        response->success = true;
        response->message = "success";
    }catch(const std::exception& e){
        
        response->success = false;
        response->message = "param override false";
    }
}

void WaypointReconfigureNode::param_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "params_reset");

    try{
        param_client_->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", prev_global_inflation_radius)});
        param_client_->set_parameters({rclcpp::Parameter("inflation_layer.cost_scaling_factor", prev_global_cost_scaling_factor)});

        response->success = true;
        response->message = "success";
    }catch(const std::exception& e){
        
        response->success = false;
        response->message = "param reset false";
    }
}

bool WaypointReconfigureNode::read_yaml()
{
    try{
        std::string yaml_pass = ament_index_cpp::get_package_share_directory("waypoint_reconfigure")+"/config/"+"nav2_params_override"+".yaml";
        RCLCPP_INFO(this->get_logger(), "yaml pass : %s", yaml_pass.c_str());
        YAML::Node yaml_config = YAML::LoadFile(yaml_pass);

        global_inflation_radius = yaml_config["waypoint_reconfigure"]["global_inflation_radius"].as<double>();
        global_cost_scaling_factor = yaml_config["waypoint_reconfigure"]["global_cost_scaling_factor"].as<double>();
        local_inflation_radius = yaml_config["waypoint_reconfigure"]["local_inflation_radius"].as<double>();
        local_cost_scaling_factor = yaml_config["waypoint_reconfigure"]["local_cost_scaling_factor"].as<double>();

        prev_global_inflation_radius = yaml_config["waypoint_reconfigure"]["prev_global_inflation_radius"].as<double>();
        prev_global_cost_scaling_factor = yaml_config["waypoint_reconfigure"]["prev_global_cost_scaling_factor"].as<double>();
        prev_local_inflation_radius = yaml_config["waypoint_reconfigure"]["prev_local_inflation_radius"].as<double>();
        prev_local_cost_scaling_factor = yaml_config["waypoint_reconfigure"]["prev_local_cost_scaling_factor"].as<double>();

        return true;
    }catch(const std::exception& e){
        return false;
    }
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    auto node = std::make_shared<waypoint_reconfigure::WaypointReconfigureNode>(options);
    bool read_result = node->read_yaml();

    if(!read_result){
        std::cerr << "read yaml error" << std::endl;
        return 1;
    }else{
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
}