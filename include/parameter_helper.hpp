/**
 * @file parameter_helper.hpp
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

#ifndef BEGINNER_TUTORIALS_INCLUDE_PARAMETER_HELPER_HPP_
#define BEGINNER_TUTORIALS_INCLUDE_PARAMETER_HELPER_HPP_

// standard libraries
#include <chrono>
#include <functional>
#include <string>

// ros libraries
#include <rclcpp/rclcpp.hpp>

// namespaces
using namespace std::chrono_literals;  // for use of time units: "ms", "s"

/**
 * @brief Class ParameterHelper
 * 
 */
class ParameterHelper : public rclcpp::Node {
 public:
  //////////////////////////
  // Constructor
  //////////////////////////
  ParameterHelper();

  //////////////////////////
  // Member Functions:
  //////////////////////////
  void timer_callback();

 private:
  rclcpp::TimerBase::SharedPtr timer_;  // timer variable
};

#endif  // BEGINNER_TUTORIALS_INCLUDE_PARAMETER_HELPER_HPP_
