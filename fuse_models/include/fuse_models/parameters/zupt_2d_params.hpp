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
#ifndef FUSE_MODELS__PARAMETERS__ZUPT_2D_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__ZUPT_2D_PARAMS_HPP_

#include <string>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/parameter.hpp>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Zupt2D class
 */
struct Zupt2DParams : public ParameterBase
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] interfaces - The node interfaces with which to load parameters
   * @param[in] ns - The parameter namespace to use
   */
  void loadFromROS(
    fuse_core::node_interfaces::NodeInterfaces<
      fuse_core::node_interfaces::Base,
      fuse_core::node_interfaces::Logging,
      fuse_core::node_interfaces::Parameters
    > interfaces,
    const std::string & ns)
  {
    disable_checks =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "disable_checks"),
      disable_checks);
    queue_size = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "queue_size"),
      queue_size);

    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "throttle_period"), throttle_period,
      false);
    throttle_use_wall_time =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "throttle_use_wall_time"),
      throttle_use_wall_time);

    fuse_core::getParamRequired(interfaces, fuse_core::joinParameterName(ns, "topic"), topic);
    topic2 = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "topic2"),
      topic2);

    velocity_threshold = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_threshold"),
      velocity_threshold);
    angular_threshold = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "angular_threshold"),
      angular_threshold);
    // The agreement topic thresholds default to the primary thresholds
    velocity_threshold2 = velocity_threshold;
    angular_threshold2 = angular_threshold;
    velocity_threshold2 = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_threshold2"),
      velocity_threshold2);
    angular_threshold2 = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "angular_threshold2"),
      angular_threshold2);
    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "topic2_timeout"), topic2_timeout,
      false);
    velocity_sigma = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_sigma"),
      velocity_sigma);
    angular_sigma = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "angular_sigma"),
      angular_sigma);
    acceleration_sigma = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "acceleration_sigma"),
      acceleration_sigma);
  }

  bool disable_checks {false};
  int queue_size {10};
  rclcpp::Duration throttle_period {0, 0};  //!< The throttle period duration in seconds
  bool throttle_use_wall_time {false};      //!< Whether to throttle using ros::WallTime or not
  std::string topic {};

  //!< Optional second odometry topic that must agree the robot is stationary before ZUPT fires.
  //!< Guards against a single degenerate odometry source (e.g. ICP in a featureless corridor)
  //!< confidently reporting zero velocity while the robot is moving. Empty = single-topic mode.
  std::string topic2 {};

  //!< The robot is considered stationary when the measured linear speed is below
  //!< velocity_threshold (m/s) AND the measured yaw rate is below angular_threshold (rad/s)
  double velocity_threshold {0.05};
  double angular_threshold {0.05};

  //!< Stationarity thresholds for the agreement topic. Odometry sources have different noise
  //!< levels, so the agreement topic can use looser/tighter values; they default to the primary
  //!< thresholds when not set.
  double velocity_threshold2 {0.05};
  double angular_threshold2 {0.05};

  //!< Maximum age of the newest topic2 sample, relative to the primary message stamp, for the
  //!< agreement to count (seconds). If topic2 is configured but stale or never received, ZUPT
  //!< does not fire: a missing confirmation reads as "not confirmed stationary".
  rclcpp::Duration topic2_timeout {0, 500000000};  // 0.5 s

  //!< Noise applied to the zero-velocity pseudo-measurements. Tight values make the optimizer
  //!< strongly believe the robot is not moving while it is stationary.
  double velocity_sigma {0.01};  //!< Linear velocity sigma (m/s)
  double angular_sigma {0.01};   //!< Yaw velocity sigma (rad/s)

  //!< Noise applied to the zero-acceleration pseudo-measurement (m/s^2). This suppresses
  //!< accelerometer noise while stationary. Set to <= 0.0 to disable the acceleration constraint.
  double acceleration_sigma {0.05};
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS__PARAMETERS__ZUPT_2D_PARAMS_HPP_