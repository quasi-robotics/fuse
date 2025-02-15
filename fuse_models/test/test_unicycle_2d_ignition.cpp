/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <utility>
#include <vector>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_models/unicycle_2d_ignition.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <fuse_msgs/srv/set_pose_deprecated.hpp>
#include <rclcpp/rclcpp.hpp>

using fuse_constraints::AbsolutePosition2DStampedConstraint;
using fuse_constraints::AbsoluteOrientation2DStampedConstraint;
using fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint;
using fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint;
using fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint;


/**
 * @brief Promise used to communicate between the tests and the callback
 */
std::promise<fuse_core::Transaction::SharedPtr> callback_promise;

/**
 * @brief Transaction callback that forwards the transaction into the promise result
 */
void transactionCallback(fuse_core::Transaction::SharedPtr transaction)
{
  callback_promise.set_value(std::move(transaction));
}

/**
 * @brief Helper function for fetching the desired constraint from a transaction
 */
template<typename Derived>
const Derived * getConstraint(const fuse_core::Transaction & transaction)
{
  for (const auto & constraint : transaction.addedConstraints()) {
    auto derived = dynamic_cast<const Derived *>(&constraint);
    if (derived) {
      return derived;
    }
  }
  return nullptr;
}


class Unicycle2DIgnitionTestFixture : public ::testing::Test
{
public:
  Unicycle2DIgnitionTestFixture()
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_ = std::thread(
      [&]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.reset();
    rclcpp::shutdown();
  }

  std::thread spinner_;   //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(Unicycle2DIgnitionTestFixture, InitialTransaction)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]"});
  auto node = rclcpp::Node::make_shared("unicycle_2d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Unicycle2DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // The ignition sensor should publish a transaction immediately. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 0.1, 1.2;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 4.0;
    auto actual = getConstraint<AbsolutePosition2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 2.3;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 9.0;
    auto actual = getConstraint<AbsoluteOrientation2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 3.4, 4.5;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 16.0, 0.0, 0.0, 25.0;
    auto actual = getConstraint<AbsoluteVelocityLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 5.6;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 36.0;
    auto actual = getConstraint<AbsoluteVelocityAngular2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 6.7, 7.8;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 64.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}

TEST_F(Unicycle2DIgnitionTestFixture, SkipInitialTransaction)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("unicycle_2d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Unicycle2DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // The ignition sensor should publish a transaction immediately. Wait for the callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(1));
  ASSERT_FALSE(status == std::future_status::ready);
}

TEST_F(Unicycle2DIgnitionTestFixture, SetPoseService)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]",
    "-p", "ignition_sensor.set_pose_service:=set_pose",
    "-p", "ignition_sensor.reset_service:=''",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("unicycle_2d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Unicycle2DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // Call the SetPose service
  auto srv = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  srv->pose.header.stamp = rclcpp::Time(12, 345678910);
  srv->pose.pose.pose.position.x = 1.0;
  srv->pose.pose.pose.position.y = 2.0;
  srv->pose.pose.pose.orientation.z = 0.0499792;  // yaw = 0.1rad
  srv->pose.pose.pose.orientation.w = 0.9987503;
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 2.0;
  srv->pose.pose.covariance[35] = 3.0;
  auto client = node->create_client<fuse_msgs::srv::SetPose>("/unicycle_2d_ignition_test/set_pose");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(srv);
  ASSERT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));
  EXPECT_TRUE(result.get()->success);

  // The ignition sensor should publish a transaction in response to the service call. Wait for the
  // callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 1.0, 2.0;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 2.0;
    auto actual = getConstraint<AbsolutePosition2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 0.1;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 3.0;
    auto actual = getConstraint<AbsoluteOrientation2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-5);  // My quaternion isn't super-exact
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 3.4, 4.5;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 16.0, 0.0, 0.0, 25.0;
    auto actual = getConstraint<AbsoluteVelocityLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 5.6;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 36.0;
    auto actual = getConstraint<AbsoluteVelocityAngular2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 6.7, 7.8;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 64.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}

TEST_F(Unicycle2DIgnitionTestFixture, SetPoseDeprecatedService)
{
  // Set some configuration
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "-p", "ignition_sensor.initial_state:="
    "[0.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8]",
    "-p", "ignition_sensor.initial_sigma:="
    "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]",
    "-p", "ignition_sensor.set_pose_deprecated_service:=set_pose_deprecated",
    "-p", "ignition_sensor.reset_service:=''",
    "-p", "ignition_sensor.publish_on_startup:=false"});
  auto node = rclcpp::Node::make_shared("unicycle_2d_ignition_test", options);
  executor_->add_node(node);

  // Initialize the callback promise. Promises are single-use.
  callback_promise = std::promise<fuse_core::Transaction::SharedPtr>();
  auto callback_future = callback_promise.get_future();

  // Create an ignition sensor and register the callback
  fuse_models::Unicycle2DIgnition ignition_sensor;
  ignition_sensor.initialize(*node, "ignition_sensor", &transactionCallback);
  ignition_sensor.start();

  // Call the SetPose service
  auto srv = std::make_shared<fuse_msgs::srv::SetPoseDeprecated::Request>();
  srv->pose.header.stamp = rclcpp::Time(12, 345678910);
  srv->pose.pose.pose.position.x = 1.0;
  srv->pose.pose.pose.position.y = 2.0;
  srv->pose.pose.pose.orientation.z = 0.0499792;  // yaw = 0.1rad
  srv->pose.pose.pose.orientation.w = 0.9987503;
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 2.0;
  srv->pose.pose.covariance[35] = 3.0;
  auto client = node->create_client<fuse_msgs::srv::SetPoseDeprecated>(
    "/unicycle_2d_ignition_test/set_pose_deprecated");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(srv);
  ASSERT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));

  // The ignition sensor should publish a transaction in response to the service call. Wait for the
  // callback to fire.
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  // Check the transaction
  auto transaction = callback_future.get();
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 1.0, 2.0;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 1.0, 0.0, 0.0, 2.0;
    auto actual = getConstraint<AbsolutePosition2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 0.1;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 3.0;
    auto actual = getConstraint<AbsoluteOrientation2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-5);  // My quaternion isn't super-exact
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 3.4, 4.5;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 16.0, 0.0, 0.0, 25.0;
    auto actual = getConstraint<AbsoluteVelocityLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector1d expected_mean;
    expected_mean << 5.6;
    fuse_core::Matrix1d expected_cov;
    expected_cov << 36.0;
    auto actual = getConstraint<AbsoluteVelocityAngular2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
  {
    fuse_core::Vector2d expected_mean;
    expected_mean << 6.7, 7.8;
    fuse_core::Matrix2d expected_cov;
    expected_cov << 49.0, 0.0, 0.0, 64.0;
    auto actual = getConstraint<AbsoluteAccelerationLinear2DStampedConstraint>(*transaction);
    ASSERT_TRUE(static_cast<bool>(actual));
    EXPECT_MATRIX_NEAR(expected_mean, actual->mean(), 1.0e-9);
    EXPECT_MATRIX_NEAR(expected_cov, actual->covariance(), 1.0e-9);
  }
}
