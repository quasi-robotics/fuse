/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#ifndef FUSE_MODELS__GRAPH_IGNITION_HPP_
#define FUSE_MODELS__GRAPH_IGNITION_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <fuse_msgs/srv/set_graph.hpp>
#include <fuse_models/parameters/graph_ignition_params.hpp>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/graph_deserializer.hpp>

#include <fuse_msgs/msg/serialized_graph.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>


namespace fuse_models
{

/**
 * @brief A ignition sensor designed to be used to reset the optimizer graph to an input graph. This
 *        is useful for debugging purposes because it allows to play back the recorded transactions
 *        from a previous run starting from the same graph, so we obtain the same intermediate
 *        graphs and publishers' outputs. This is specially useful when we cannot record all the
 *        graph messages because that would take too much bandwidth or disk, so the recorded graph
 *        must be throttled.
 *
 * This class publishes a transaction equivalent to the supplied graph. When the sensor is first
 * loaded, it does not send any transactions. It waits for a graph to do so. Whenever a graph is
 * received, either on the set_graph service or the topic, this ignition sensor resets the optimizer
 * then publishes a new transaction equivalent to the specified graph.
 *
 * Parameters:
 *  - ~queue_size (int, default: 10) The subscriber queue size for the graph messages
 *  - ~reset_service (string, default: "~/reset") The name of the reset service to call before
 *                                                sending a transaction
 *  - ~set_graph_service (string, default: "set_graph") The name of the set_graph service to
 *                                                      advertise
 *  - ~topic (string, default: "graph") The topic name for received Graph messages
 */
class GraphIgnition : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(GraphIgnition)
  using ParameterType = parameters::GraphIgnitionParams;

  /**
   * @brief Default constructor
   *
   * All plugins are required to have a constructor that accepts no arguments
   */
  GraphIgnition();

  /**
   * @brief Destructor
   */
  ~GraphIgnition() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   *
   * As a very special case, we are overriding the start() method instead of providing an onStart()
   * implementation. This is because the GraphIgnition sensor calls reset() on the optimizer, which
   * in turn calls stop() and start(). If we used the AsyncSensorModel implementations of start()
   * and stop(), the system would hang inside of one callback function while waiting for another
   * callback to complete.
   */
  void start() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   *
   * As a very special case, we are overriding the stop() method instead of providing an onStop()
   * implementation. This is because the GraphIgnition sensor calls reset() on the optimizer, which
   * in turn calls stop() and start(). If we used the AsyncSensorModel implementations of start()
   * and stop(), the system would hang inside of one callback function while waiting for another
   * callback to complete.
   */
  void stop() override;

protected:
  /**
   * @brief Triggers the publication of a new transaction equivalent to the supplied graph
   */
  void subscriberCallback(const fuse_msgs::msg::SerializedGraph & msg);

  /**
   * @brief Triggers the publication of a new transaction equivalent to the supplied graph
   */
  bool setGraphServiceCallback(
    rclcpp::Service<fuse_msgs::srv::SetGraph>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> request_id,
    const fuse_msgs::srv::SetGraph::Request::SharedPtr req);

  /**
   * @brief Perform any required initialization for the kinematic ignition sensor
   */
  void onInit() override;

  /**
   * @brief Process a received graph from one of the ROS comm channels
   *
   * This method validates the input graph, resets the optimizer, then constructs and sends the
   * initial state constraints (by calling sendGraph()).
   *
   * @param[in] msg - The graph message
   */
  void process(
    const fuse_msgs::msg::SerializedGraph & msg, std::function<void()> post_process = nullptr);

  /**
   * @brief Create and send a transaction equivalent to the supplied graph
   *
   * @param[in] graph - The graph
   * @param[in] stamp - The graph stamp
   */
  void sendGraph(const fuse_core::Graph & graph, const rclcpp::Time & stamp);

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Graph,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Services,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  std::atomic_bool started_;  //!< Flag indicating the sensor has been started
  rclcpp::Logger logger_;  //!< The sensor model's logger

  ParameterType params_;  //!< Object containing all of the configuration parameters

  //!< Service client used to call the "reset" service on the optimizer
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Service<fuse_msgs::srv::SetGraph>::SharedPtr set_graph_service_;
  rclcpp::Subscription<fuse_msgs::msg::SerializedGraph>::SharedPtr sub_;

  fuse_core::GraphDeserializer graph_deserializer_;  //!< Deserializer for SerializedGraph messages
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__GRAPH_IGNITION_HPP_
