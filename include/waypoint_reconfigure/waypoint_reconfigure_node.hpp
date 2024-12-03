#pragma once

#include <exception>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include "waypoint_reconfigure/visibility_control.h"

namespace waypoint_reconfigure{

class WaypointReconfigureNode : public rclcpp::Node {
public:
  WAYPOINT_RECONFIGURE_PUBLIC
  explicit WaypointReconfigureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  WAYPOINT_RECONFIGURE_PUBLIC
  explicit WaypointReconfigureNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void param_override(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response
  );

  bool read_yaml();

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr param_change_srv_;

  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

private:
  double global_inflation_radius;
  double global_cost_scaling_factor;
  double local_inflation_radius;
  double local_cost_scaling_factor;

  double prev_global_inflation_radius;
  double prev_global_cost_scaling_factor;
  double prev_local_inflation_radius;
  double prev_local_cost_scaling_factor;
};

}