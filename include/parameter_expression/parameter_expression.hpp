// Copyright 2025 ForteFibre
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

#pragma once

#include <muParser.h>

#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_event_handler.hpp>

namespace parameter_expression
{
// ROS 2 dynamic typed parameter with mathematical expression
class ParameterExpression
{
public:
  ParameterExpression(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
    const std::string & name, const double default_value,
    rcl_interfaces::msg::ParameterDescriptor descriptor);

  template <typename Node>
  ParameterExpression(
    Node & node, const std::string & name, const double default_value = 0.0,
    rcl_interfaces::msg::ParameterDescriptor descriptor =
      rcl_interfaces::msg::ParameterDescriptor())
  : ParameterExpression(
      rclcpp::node_interfaces::get_node_parameters_interface(node), name, default_value, descriptor)
  {
  }

  virtual ~ParameterExpression();

  double get() const;

  RCLCPP_SMART_PTR_DEFINITIONS(ParameterExpression)

private:
  rcl_interfaces::msg::SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> &);
  void eval(const rclcpp::Parameter parameter_value);

  void eval_first();

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::Parameter parameter_;
  const std::string name_;
  mu::Parser parser_;
  double value_;
  const double default_value_;
};
}  // namespace parameter_expression