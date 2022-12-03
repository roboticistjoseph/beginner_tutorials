/**
 * @file publisher_node.hpp
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

#ifndef BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_
#define BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_

// standard libraries
#include <chrono>
#include <string>
#include <memory>

// ros libraries
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// service includes
#include "example_interfaces/srv/add_two_ints.hpp"

// tf libraries
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// namespaces
using namespace std::chrono_literals;  // for use of time units: "ms", "s"
using std::placeholders::_1;           // for use with binding Class member
using std::placeholders::_2;           // callback function

// topic types
using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

// service types
using SERVICE = rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr;
using ADDTWOINTS = example_interfaces::srv::AddTwoInts;
using REQUEST =
   const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>;
using RESPOSE = std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>;

/**
 * @brief Publisher class.
 * 
 */
class Publisher : public rclcpp::Node {
 public:
  //////////////////////////
  // Constructor
  //////////////////////////
    Publisher();

 private:
  //////////////////////////
  // Member Variables:
  //////////////////////////
  size_t    m_count_;
  PUBLISHER m_publisher_;
  TIMER     m_timer_;
  SERVICE   m_service_;

  //////////////////////////
  // Member Functions:
  //////////////////////////

  /**
   * @brief Timer Callback
   * 
   */
  void timer_callback();

  /**
   * @brief Function to add two numbers from request
   * 
   * @param request 
   * @param response 
   */
  void add(REQUEST request, RESPOSE response);

  /**
   * @brief Timer variable
   * 
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief publisher variable
   * 
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief counter variable
   * 
   */
  size_t count_;

  /**
   * @brief variable for Static tf broadcaster
   * 
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

#endif  // BEGINNER_TUTORIALS_INCLUDE_PUBLISHER_NODE_HPP_
