// Copyright (c) 2025 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_ROS_COMMON__ACTION_CLIENT_HPP_
#define NAV2_ROS_COMMON__ACTION_CLIENT_HPP_

#include <string>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace nav2
{

/**
 * @class nav2::ActionClient
 * @brief A simple wrapper on ROS2 action client
 */
template<typename ActionT, typename NodeT = rclcpp::Node::SharedPtr>
class ActionClient
{
public:
  using SharedPtr = std::shared_ptr<nav2::ActionClient<ActionT, NodeT>>;
  using UniquePtr = std::unique_ptr<nav2::ActionClient<ActionT, NodeT>>;

  /**
  * @brief A constructor
  * @param action_name name of the action to call
  * @param provided_node Node to create the action client off of
  * @param callback_group Callback group to use for the action client
  */
  explicit ActionClient(
    const std::string & action_name,
    const NodeT & provided_node,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : action_name_(action_name), node_(provided_node), callback_group_(callback_group)
  {
    // When a nullptr is passed, the client will use the default callback group
    client_ = rclcpp_action::create_client<ActionT>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      action_name_,
      callback_group_);

    rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    if (!node_->has_parameter("service_introspection_mode")) {
      node_->declare_parameter("service_introspection_mode", "disabled");
    }
    std::string service_introspection_mode =
      node_->get_parameter("service_introspection_mode").as_string();
    if (service_introspection_mode == "metadata") {
      introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
    } else if (service_introspection_mode == "contents") {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    }

    this->client_->configure_introspection(
        node_->get_clock(), rclcpp::ServicesQoS(), introspection_state);
  }

protected:
  std::string action_name_;
  NodeT node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__ACTION_CLIENT_HPP_
