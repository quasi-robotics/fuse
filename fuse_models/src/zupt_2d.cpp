/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Quasi, Inc.
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
#include <cmath>
#include <vector>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/zupt_2d.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Zupt2D, fuse_core::SensorModel)

namespace fuse_models
{

Zupt2D::Zupt2D()
: fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  throttled_callback_(std::bind(&Zupt2D::process, this, std::placeholders::_1))
{
}

void Zupt2D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void Zupt2D::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (!params_.throttle_use_wall_time) {
    throttled_callback_.setClock(clock_);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
}

void Zupt2D::onStart()
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    interfaces_,
    params_.topic,
    params_.queue_size,
    std::bind(
      &OdometryThrottledCallback::callback<const nav_msgs::msg::Odometry &>,
      &throttled_callback_,
      std::placeholders::_1
    ),
    sub_options
  );
}

void Zupt2D::onStop()
{
  sub_.reset();
}

void Zupt2D::process(const nav_msgs::msg::Odometry & msg)
{
  // Detect stationarity from the raw odometry measurement, NOT from the optimized state: the
  // measured twist is the ground truth for "the wheels are not turning", while the estimated
  // velocity is exactly the quantity being corrupted by the IMU noise this sensor suppresses.
  const double speed = std::hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
  const double yaw_rate = std::abs(msg.twist.twist.angular.z);

  if (speed >= params_.velocity_threshold || yaw_rate >= params_.angular_threshold) {
    return;
  }

  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.header.stamp);

  // Assert [VX = 0, VY = 0, VYAW = 0] with tight noise. The zero pseudo-measurement is generated
  // directly in the odometry twist frame (the robot body frame), so no transform is required.
  // No loss function is used: this constraint only fires when the odometry already confirms the
  // robot is stationary, so down-weighting it as an outlier would defeat its purpose.
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header = msg.header;
  twist.twist.covariance[0] = params_.velocity_sigma * params_.velocity_sigma;
  twist.twist.covariance[7] = params_.velocity_sigma * params_.velocity_sigma;
  twist.twist.covariance[35] = params_.angular_sigma * params_.angular_sigma;

  const std::vector<size_t> linear_indices {0, 1};   // vx, vy
  const std::vector<size_t> angular_indices {0};     // vyaw

  common::processTwistWithCovariance(
    name(),
    device_id_,
    twist,
    fuse_core::Loss::SharedPtr(),
    fuse_core::Loss::SharedPtr(),
    "",  // no target frame: use the pseudo-measurement as-is
    linear_indices,
    angular_indices,
    *tf_buffer_,
    !params_.disable_checks,
    *transaction);

  // Assert [AX = 0, AY = 0] to suppress accelerometer noise at its source while stationary
  if (params_.acceleration_sigma > 0.0) {
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header = msg.header;
    accel.accel.covariance[0] = params_.acceleration_sigma * params_.acceleration_sigma;
    accel.accel.covariance[7] = params_.acceleration_sigma * params_.acceleration_sigma;

    const std::vector<size_t> acceleration_indices {0, 1};  // ax, ay

    common::processAccelWithCovariance(
      name(),
      device_id_,
      accel,
      fuse_core::Loss::SharedPtr(),
      "",  // no target frame: use the pseudo-measurement as-is
      acceleration_indices,
      *tf_buffer_,
      !params_.disable_checks,
      *transaction);
  }

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

}  // namespace fuse_models
