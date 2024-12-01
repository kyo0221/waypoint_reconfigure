#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "waypoint_reconfigure/visibility_control.h"

namespace waypoint_reconfigure{

class WaypointReconfigureNode : public rclcpp::Node {
public:
  WAYPOINT_RECONFIGURE_PUBLIC
  explicit WaypointReconfigureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  WAYPOINT_RECONFIGURE_PUBLIC
  explicit WaypointReconfigureNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool param_override(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response
  );

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr param_change_srv_;

  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
};

}