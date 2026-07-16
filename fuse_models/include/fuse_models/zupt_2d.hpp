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
#ifndef FUSE_MODELS__ZUPT_2D_HPP_
#define FUSE_MODELS__ZUPT_2D_HPP_

#include <memory>
#include <string>

#include <fuse_models/parameters/zupt_2d_params.hpp>
#include <fuse_core/throttled_callback.hpp>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>


namespace fuse_models
{

/**
 * @brief A sensor model that produces zero-velocity update (ZUPT) constraints when the robot is
 *        stationary
 *
 * This sensor subscribes to a nav_msgs::msg::Odometry topic (typically the wheel odometry) and,
 * whenever the measured twist indicates the robot is stationary, generates tight zero-mean
 * absolute constraints on the linear velocity, yaw velocity, and (optionally) linear acceleration
 * variables. This suppresses IMU accelerometer/gyro noise from drifting the state estimate while
 * the robot is not moving.
 *
 * Stationarity is detected from the raw odometry measurement, never from the optimized state: the
 * measured twist is the ground truth for "the wheels are not turning", while the estimated
 * velocity is exactly the quantity being corrupted by IMU noise.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                           publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - queue_size (int, default: 10) The subscriber queue size for the odometry messages
 *  - topic (string) The topic to which to subscribe for the odometry messages
 *  - topic2 (string, default: "") Optional second odometry topic that must also report the robot
 *                                 stationary before ZUPT fires. Guards against a single degenerate
 *                                 odometry source (e.g. ICP in a featureless corridor) confidently
 *                                 reporting zero velocity while the robot is moving. Empty
 *                                 disables the agreement check
 *  - velocity_threshold2 (double, default: velocity_threshold) Stationarity speed threshold (m/s)
 *                                                              for topic2
 *  - angular_threshold2 (double, default: angular_threshold) Stationarity yaw rate threshold
 *                                                            (rad/s) for topic2
 *  - topic2_timeout (double, default: 0.5) Maximum age (s) of the newest topic2 sample, relative
 *                                          to the primary message stamp, for the agreement to
 *                                          count. A stale or missing topic2 blocks ZUPT
 *  - velocity_threshold (double, default: 0.05) Measured linear speed (m/s) below which the robot
 *                                               may be considered stationary
 *  - angular_threshold (double, default: 0.05) Measured yaw rate (rad/s) below which the robot may
 *                                              be considered stationary
 *  - velocity_sigma (double, default: 0.01) Noise sigma (m/s) on the zero linear velocity
 *                                           pseudo-measurement
 *  - angular_sigma (double, default: 0.01) Noise sigma (rad/s) on the zero yaw velocity
 *                                          pseudo-measurement
 *  - acceleration_sigma (double, default: 0.05) Noise sigma (m/s^2) on the zero linear
 *                                               acceleration pseudo-measurement; <= 0.0 disables
 *                                               the acceleration constraint
 *
 * Subscribes:
 *  - \p topic (nav_msgs::msg::Odometry) Odometry used to detect that the robot is stationary
 *  - \p topic2 (nav_msgs::msg::Odometry) Optional odometry that must agree with the detection
 */
class Zupt2D : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Zupt2D)
  using ParameterType = parameters::Zupt2DParams;

  /**
   * @brief Default constructor
   */
  Zupt2D();

  /**
   * @brief Destructor
   */
  virtual ~Zupt2D() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Callback for odometry messages
   * @param[in] msg - The odometry message used to detect that the robot is stationary
   */
  void process(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief Callback for the optional agreement odometry topic
   * @param[in] msg - The odometry message used to confirm that the robot is stationary
   */
  void process2(const nav_msgs::msg::Odometry & msg);

protected:
  fuse_core::UUID device_id_;  //!< The UUID of this device

  /**
   * @brief Loads ROS parameters and subscribes to the parameterized topic
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   */
  void onStop() override;

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  rclcpp::Clock::SharedPtr clock_;  //!< The sensor model's clock, for timestamping and logging
  rclcpp::Logger logger_;  //!< The sensor model's logger

  ParameterType params_;

  //!< A transform buffer is required by the sensor_proc helpers, but the zero pseudo-measurements
  //!< are generated directly in the twist frame, so no transform lookup is ever performed and no
  //!< transform listener is needed.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub2_;

  //!< Latest agreement-topic sample. Both subscriptions share the sensor model's mutually
  //!< exclusive callback group, so callbacks are serialized and no locking is needed.
  rclcpp::Time last_topic2_stamp_ {0, 0, RCL_ROS_TIME};
  bool last_topic2_stationary_ {false};
  bool topic2_received_ {false};

  using OdometryThrottledCallback = fuse_core::ThrottledMessageCallback<nav_msgs::msg::Odometry>;
  OdometryThrottledCallback throttled_callback_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__ZUPT_2D_HPP_
