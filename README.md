# `parameter_expression`

A ROS 2 package that allows using mathematical expressions in dynamic parameters using the [muParser library](https://beltoforion.de/en/muparser/).

## Overview

The `parameter_expression` package provides a C++ class that enables ROS 2 nodes to use mathematical expressions as parameter values. Instead of static numeric values, you can use expressions like `"sin(0.5) * 2 + 1"` which will be evaluated dynamically when the parameter is set.

## Features

- **Dynamic Expression Evaluation**: Parameters can be set as mathematical expressions that are evaluated in real-time
- **Mathematical Functions**: Supports common mathematical functions like sin, cos, tan, atan2, etc (powered by muParser).
- **Type Flexibility**: Automatically handles different parameter types (int, double, string expressions)
- **Error Handling**: Provides error messages for invalid expressions

## Dependencies

- **rclcpp**: ROS 2 C++ client library
- **muParser**: Mathematical expression parser library

## Usage

### Basic Usage

Include the header in your ROS 2 node:

```cpp
#include <parameter_expression/parameter_expression.hpp>
```

Also, ensure you have the necessary dependencies in your `package.xml` and `CMakeLists.txt`:

```xml
<depend>parameter_expression</depend>
```

```cmake
find_package(parameter_expression REQUIRED)

target_link_libraries(
  your_node
  parameter_expression::parameter_expression
)
```

Create a parameter expression in your node:

```cpp
class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Create a parameter expression with default value 0.0
    expression_param_ = std::make_shared<parameter_expression::ParameterExpression>(*this, "my_expression", 0.0);
  }

  void some_function()
  {
    // Get the current evaluated value
    double value = expression_param_->get();
    RCLCPP_INFO(this->get_logger(), "Expression value: %f", value);
  }

private:
  parameter_expression::ParameterExpression::SharedPtr expression_param_;
};
```

### Setting Parameters

You can set the parameter using various methods:

#### Via Command Line
```bash
ros2 param set /my_node my_expression "sin(3.14159/2) * 100"
```

#### Via Launch File or parameter YAML file
```xml
<launch>
  <node pkg="your_package" exec="your_node" name="my_node">
    <param name="my_expression" value="2 * _pi * 0.5" />
  </node>
</launch>
```


### Advanced Usage

#### Custom Parameter Descriptor
```cpp
rcl_interfaces::msg::ParameterDescriptor descriptor;
descriptor.description = "A mathematical expression for calculation";
descriptor.additional_constraints = "Must be a valid mathematical expression";

auto expression_param = std::make_shared<parameter_expression::ParameterExpression>(
  *this, "advanced_expression", 1.0, descriptor);
```

#### Using with Node Parameters Interface
```cpp
auto param_interface = rclcpp::node_interfaces::get_node_parameters_interface(node);
auto expression_param = std::make_shared<parameter_expression::ParameterExpression>(
  param_interface, "expression_name", 0.0, descriptor);
```

## Supported Mathematical Operations

The package supports all operations provided by the muParser library: [muParser documentation](https://beltoforion.de/en/muparser/features.php).

## Error Handling

The package provides comprehensive error handling:

- **Invalid Expressions**: Returns error with description for malformed expressions
- **Type Errors**: Handles incorrect parameter types gracefully
- **Parser Errors**: Provides detailed error messages from muParser

### Set invalid parameter will return an error
```cpp
// This will set the parameter but return an error result
auto result = node->set_parameter(rclcpp::Parameter("my_expression", "invalid_expr+"));
if (!result.successful) {
  RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
}
```

## License

This package is licensed under the Apache License 2.0.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request


### Testing

Run the tests to verify functionality:

```bash
colcon test --packages-select parameter_expression
colcon test-result --verbose
```

The test suite covers:
- Basic mathematical operations
- Function evaluations
- Type conversions
- Error handling
- Parameter setting via different methods

For contributors, please ensure all tests pass and add new tests for any new features or changes.

## Examples
See the test files in the `test/` directory for comprehensive usage examples.
