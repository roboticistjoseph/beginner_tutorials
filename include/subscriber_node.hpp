/**
 * @file subscriber_node.hpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@terpmail.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright 
 *  // Copyright 2016 Open Source Robotics Foundation, Inc.
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
    //////////////////////////
    // Constructor
    //////////////////////////
    Subscriber();

 private:
  //////////////////////////
  // Member Functions:
  //////////////////////////

  /**
   * @brief callback function
   * 
   * @param msg 
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

  /**
   * @brief Subscriber variable
   * 
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // BEGINNER_TUTORIALS_INCLUDE_SUBSCRIBER_NODE_HPP_
