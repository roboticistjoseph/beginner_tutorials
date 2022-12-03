/**
 * @file publisher_node.cpp
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

// header files
#include "../include/publisher_node.hpp"

Publisher::Publisher()
    : Node("publisher"), count_(0) {
      /* 
       * Create publisher with buffer size of 10 and frequency = 2 hz
       */
       auto topicName = "topic";
       publisher_ =
        this->create_publisher<std_msgs::msg::String>(topicName, 10);
       auto topicCallbackPtr = std::bind(&Publisher::timer_callback, this);
       timer_ = this->create_wall_timer(500ms, topicCallbackPtr);

       /*
        * Creates a service server (with a service name = "add_two_ints_v2")
        */
       auto serviceName = "add_two_ints_v2";
       auto serviceCallbackPtr = std::bind(&Publisher::add, this, _1, _2);
       m_service_ =
        create_service <ADDTWOINTS>(serviceName, serviceCallbackPtr);

      // publish_message_ = "Custom String Message for Printing";
       /**
        * @brief tf broadcaster
        * 
        */
       tf_static_broadcaster_ = std::make_shared<
                tf2_ros::StaticTransformBroadcaster>(this);

       /**
        * @brief obect for transformation matrix
        * 
        */
       geometry_msgs::msg::TransformStamped t;

       // starting clock
       t.header.stamp = this->get_clock()->now();
       // parent frame
       t.header.frame_id = "world";
       // child frame
       t.child_frame_id = "talk";

       // Translation along x, y and z axis
       t.transform.translation.x = 0.6;
       t.transform.translation.y = 0.1;
       t.transform.translation.z = 0.9;

       // Converting Euler angles to Quaternion along XYZ
       tf2::Quaternion q;
       q.setRPY(0.6, 0.1, 0.9);
       t.transform.rotation.x = q.x();
       t.transform.rotation.y = q.y();
       t.transform.rotation.z = q.z();
       t.transform.rotation.w = q.w();

      //  // Rotation along x, y and z axis
      //  t.transform.rotation.x = 0.0523491;
      //  t.transform.rotation.y = 0.0473595;
      //  t.transform.rotation.z = 0.0523491;
      //  t.transform.rotation.w = 0.9961306;

       // broadcast
       tf_static_broadcaster_->sendTransform(t);
}

void Publisher::timer_callback() {
    // Create the message to publish
    auto message = std_msgs::msg::String();
    message.data = "Developer- Joseph:  "
             + std::to_string(count_++);

    // Argument/Parameter for setting publishing message
    auto custom_message = rcl_interfaces::msg::ParameterDescriptor();
    custom_message.description = "Name to be published: ";
    this->declare_parameter("custom_message", 2.0, custom_message);
    auto custom_msg = this->get_parameter("custom_message")
                .get_parameter_value().get<std_msgs::msg::String()>;


    RCLCPP_INFO(this->get_logger(),
                 "Publishing: '%s'", message.data.c_str());

    RCLCPP_INFO(this->get_logger(),
                 "Publishing: '%s'", custom_msg.data.c_str());

    // Publish the message
    publisher_->publish(message);
    publisher_->publish(custom_msg);

    // Logger level- Debug
    // (Debug messages detail the entire step-by-step process
    // of the system execution.)
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                         "Logger level: Debug, Publisher processing");

    // Logger level- Info
    // (Info messages indicate event and status updates that serve as a
    // visual verification that the system is running as expected.)
    RCLCPP_INFO_STREAM(this->get_logger(),
               "Logger level: Info, Publishing:" << message.data.c_str());

    // Logger level - Warning
    // (Warn messages indicate unexpected activity or non-ideal results
    // that might represent a deeper issue,
    // but don’t harm functionality outright.)
    if (count_ > 3) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                       "Logger level: Warning, Too many Publising cycles");
      }

    // Logger level - Fatal
    // (Fatal messages indicate the system is going to terminate
    // to try to protect itself from detriment.)
    if (count_ > 7) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                       "Logger level: Fatal, Fatal error due to overuse");
    }
}

void Publisher::add(REQUEST request,
            RESPOSE response) {
    response->sum = request->a + request->b;

    // Logger level- Info
    RCLCPP_INFO(this->get_logger(),
             "Incoming request\na: ", request->a, request->b);
    RCLCPP_INFO(this->get_logger(),
               "sending back response: ", (int64_t) response->sum);

    if (response->sum != request->a + request->b) {
      // Logger level - Error
      // (Error messages indicate significant issues that won’t
      // necessarily damage the system,
      // but are preventing it from functioning properly.)
      RCLCPP_ERROR_STREAM(this->get_logger(),
                           "Logger level: Error, Incorrect Calculation"
                          << std::to_string(response->sum));
      return;
  }
}

int main(int argc, char * argv[]) {
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<Publisher>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}