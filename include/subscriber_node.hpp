/**
 * @file subscriber_node.hpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@terpmail.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef BEGINNER_TUTORIALS_INCLUDE_SUBSCRIBER_NODE_HPP_
#define BEGINNER_TUTORIALS_INCLUDE_SUBSCRIBER_NODE_HPP_

// standard library
#include <memory>

// ros libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Subscriber class
 * 
 */
class Subscriber : public rclcpp::Node {
 public:
    Subscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // BEGINNER_TUTORIALS_INCLUDE_SUBSCRIBER_NODE_HPP_
