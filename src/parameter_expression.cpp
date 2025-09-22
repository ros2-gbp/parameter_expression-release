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

#include "parameter_expression/parameter_expression.hpp"

#include <muParser.h>

namespace parameter_expression
{
ParameterExpression::ParameterExpression(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  const std::string & name, const double default_value,
  rcl_interfaces::msg::ParameterDescriptor descriptor)
: node_parameters_interface_(node_parameters_interface),
  name_(name),
  parser_(),
  value_(),
  default_value_(default_value)
{
  using namespace std::placeholders;
  const auto parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(descriptor).set__dynamic_typing(true);
  const auto empty_value = rclcpp::ParameterValue();
  node_parameters_interface_->declare_parameter(name, empty_value, parameter_descriptor);
  callback_handle_ = node_parameters_interface_->add_on_set_parameters_callback(
    std::bind(&ParameterExpression::on_parameter, this, _1));
  eval_first();
}

ParameterExpression::~ParameterExpression() {}

void ParameterExpression::eval_first()
{
  const auto parameter = node_parameters_interface_->get_parameter(name_);
  eval(parameter);
}

rcl_interfaces::msg::SetParametersResult ParameterExpression::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_) {
      try {
        eval(parameter);
      } catch (const std::exception & e) {
        return rcl_interfaces::msg::SetParametersResult()
          .set__reason(std::string("Failed to evaluate expression: ") + e.what())
          .set__successful(false);
      } catch (const mu::Parser::exception_type & e) {
        return rcl_interfaces::msg::SetParametersResult()
          .set__reason(std::string("Failed to evaluate expression: ") + e.GetMsg())
          .set__successful(false);
      }
    }
  }
  return rcl_interfaces::msg::SetParametersResult().set__successful(true);
}

void ParameterExpression::eval(const rclcpp::Parameter parameter_value)
{
  using ParameterType = rcl_interfaces::msg::ParameterType;
  const auto ty = parameter_value.get_type();
  if (ty == ParameterType::PARAMETER_NOT_SET) {
    value_ = default_value_;
    return;
  }
  // If the parameter is double or integer, return the value directly
  if (ty == ParameterType::PARAMETER_INTEGER) {
    value_ = static_cast<double>(parameter_value.as_int());
    return;
  }
  if (ty == ParameterType::PARAMETER_DOUBLE) {
    value_ = parameter_value.as_double();
    return;
  }

  // If the parameter is string, parse the expression
  if (ty != ParameterType::PARAMETER_STRING) {
    throw std::runtime_error("Parameter type is not string");
  }
  const auto expression = parameter_value.as_string();

  parser_.SetExpr(expression);
  value_ = parser_.Eval();
}

double ParameterExpression::get() const { return value_; }
}  // namespace parameter_expression