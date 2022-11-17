/**
 * @file publisher_node.hpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@terpmail.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_
#define BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_

// standard libraries
#include <chrono>
#include <memory>

// ros libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// namespaces
using namespace std::chrono_literals;

/**
 * @brief Publisher class.
 * 
 */
class Publisher : public rclcpp::Node {
 public:
    Publisher();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

// RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Talker)

#endif  // BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_
