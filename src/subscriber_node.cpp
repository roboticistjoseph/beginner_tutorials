/**
 * @file subscriber_node.cpp
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

#include "../include/subscriber_node.hpp"

Subscriber::Subscriber()
  : Node("subscriber") {
     /* 
      * Create subscriber with buffer size of 10
      */
     subscription_ = this->create_subscription<std_msgs::msg::String>(
       "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
}

void Subscriber::topic_callback(
  const std_msgs::msg::String::SharedPtr msg) const {
  // Getting the message from subscribe topic
  RCLCPP_INFO(this->get_logger(), "Incoming messgae: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[]) {
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<Subscriber>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
