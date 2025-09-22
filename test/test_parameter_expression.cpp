#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "parameter_expression/parameter_expression.hpp"

class ParameterExpressionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    receiver_ = std::make_shared<rclcpp::Node>("parameter_expression_receiver");
    expression_ =
      std::make_shared<parameter_expression::ParameterExpression>(receiver_, "expression", 123.4);

    sender_ = std::make_shared<rclcpp::Node>("parameter_expression_sender");
    parameters_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(sender_, "parameter_expression_receiver");

    executor_->add_node(sender_);
    executor_->add_node(receiver_);
  }

  void TearDown() override
  {
    executor_->remove_node(sender_);
    executor_->remove_node(receiver_);
    sender_.reset();
    receiver_.reset();
  }

  template <typename TInput>
  void testExpression(const TInput expression, double expected)
  {
    using namespace std::chrono_literals;
    std::vector<rclcpp::Parameter> parameters = {rclcpp::Parameter("expression", expression)};
    auto future = parameters_client_->set_parameters(parameters);
    executor_->spin_until_future_complete(future, 500ms);
    EXPECT_DOUBLE_EQ(expression_->get(), expected)
      << "Expression: " << expression << " should be evaluated as " << expected;
  }

  template <typename TInput>
  void testError(const TInput expression)
  {
    using namespace std::chrono_literals;
    std::vector<rclcpp::Parameter> parameters = {rclcpp::Parameter("expression", expression)};
    auto future = parameters_client_->set_parameters(parameters);
    executor_->spin_until_future_complete(future, 500ms);
    EXPECT_FALSE(future.get().at(0).successful)
      << "Expression: " << expression << " should be evaluated as error";
  }

  rclcpp::Node::SharedPtr sender_, receiver_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  // Sender
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;

  // Receiver
  parameter_expression::ParameterExpression::SharedPtr expression_;
  double value_;
};

TEST_F(ParameterExpressionTest, testDefault) { EXPECT_DOUBLE_EQ(expression_->get(), 123.4); }

TEST_F(ParameterExpressionTest, testConstant) { testExpression(1.0, 1.0); }

TEST_F(ParameterExpressionTest, testInteger)
{
  testExpression("1", 1);
  testExpression("1+2", 3);
  testExpression("1-2", -1);
  testExpression("1*2", 2);
  testExpression("1/2", 0.5);
  testExpression("1+2*3", 7);
  testExpression("(1+2)*3", 9);
}

TEST_F(ParameterExpressionTest, testDouble)
{
  testExpression("1.0", 1.0);
  testExpression("1.0+2.0", 3.0);
  testExpression("1.0-2.0", -1.0);
  testExpression("1.0*2.0", 2.0);
  testExpression("1.0/2.0", 0.5);
  testExpression("1.0+2.0*3.0", 7.0);
  testExpression("(1.0+2.0)*3.0", 9.0);
}

// function
TEST_F(ParameterExpressionTest, testFunction)
{
  testExpression("sin(0)", 0);
  testExpression("cos(0)", 1);
  testExpression("tan(0)", 0);
  testExpression("atan2(0, 1)", 0);
}

// change type
TEST_F(ParameterExpressionTest, testChangeType)
{
  testExpression(123, 123);
  testExpression(123.0, 123.0);
  testExpression("123", 123);
  testExpression("121+2", 123);
}

// error
TEST_F(ParameterExpressionTest, testError)
{
  testError("1+");
  testError("1+*2");
  testError("1+2)");
  testError("1+2(");
}

class OptionsPassedParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void TearDown() override { receiver_.reset(); }

  template <typename T>
  void test(const T value, const double expected)
  {
    const auto node_options = rclcpp::NodeOptions().append_parameter_override("expression", value);
    receiver_ = std::make_shared<rclcpp::Node>("parameter_expression_receiver", node_options);
    expression_ =
      std::make_shared<parameter_expression::ParameterExpression>(receiver_, "expression", 0.0);

    rclcpp::spin_some(receiver_);

    EXPECT_DOUBLE_EQ(expression_->get(), expected)
      << "Expression: " << value << " should be evaluated as " << expected;
  }

  rclcpp::Node::SharedPtr receiver_;

  // Receiver
  parameter_expression::ParameterExpression::SharedPtr expression_;
};

TEST_F(OptionsPassedParameterTest, testConstant) { test(1.0, 1.0); }

TEST_F(OptionsPassedParameterTest, testExpression) { test("1+2", 3); }

TEST_F(OptionsPassedParameterTest, testError)
{
  // In case of passing invalid value in initialize, it should throw an
  // exception
  EXPECT_THROW(test("1+", 0), mu::Parser::exception_type);
}
