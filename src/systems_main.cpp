// Copyright 2021 Intelligent Robotics Lab
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


#include <string>

#include "lifecycle_msgs/msg/transition.hpp"

#include "attention_system/AttentionServerNode.hpp"
#include "perception_system/PeopleDetectionNode.hpp"
#include "perception_system/ObjectsDetectionNode.hpp"
#include "navigation_system/NavigationSystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


enum RtType {RT, NORT};
struct SchedNode
{
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node;
  RtType type;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::ExecutorOptions exe_rt_options;
  rclcpp::executors::MultiThreadedExecutor executor_rt(exe_rt_options, 4);
  rclcpp::executors::MultiThreadedExecutor executor_nort(rclcpp::ExecutorOptions(), 4);

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  auto aux_node = rclcpp::Node::make_shared("systems_bringup");

  // Creating of all systems nodes
  std::list<SchedNode> sched_nodes = {
    {std::make_shared<attention_system::AttentionServerNode>(node_options), RT},
    {std::make_shared<perception_system::PeopleDetectionNode>(node_options), RT},
    {std::make_shared<perception_system::ObjectsDetectionNode>(node_options), RT},
    {std::make_shared<navigation_system::NavigationSystem>(node_options), NORT},
  };

  // Adding systems nodes to the appropiate executor, depending of its RtType
  for (auto & sched_node : sched_nodes) {
    switch (sched_node.type) {
      case RT:
        executor_rt.add_node(sched_node.node->get_node_base_interface());
        break;
      case NORT:
        executor_nort.add_node(sched_node.node->get_node_base_interface());
        break;
    }
  }

  // Change systems nodes state to Inactive
  RCLCPP_INFO(aux_node->get_logger(), "Configuring systems nodes");
  for (auto & sched_node : sched_nodes) {
    sched_node.node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }
  RCLCPP_INFO(aux_node->get_logger(), "Finished configuring systems nodes");

  // Spinnning systems nodes in their executors
  RCLCPP_INFO(aux_node->get_logger(), "Executing Systems");
  auto realtime_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 60;
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) == -1) {
        perror("pthread_setschedparam failed");
        exit(-1);
      }
      executor_rt.spin();
    });

  auto no_realtime_thread = std::thread(
    [&]() {
      executor_nort.spin();
    });

  rclcpp::spin(aux_node);

  // Clean exit
  RCLCPP_INFO(aux_node->get_logger(), "Finished executing Systems - waiting for clean exit");
  no_realtime_thread.join();
  realtime_thread.join();

  rclcpp::shutdown();
  RCLCPP_INFO(aux_node->get_logger(), "Finished executing Systems - exiting");

  return 0;
}
