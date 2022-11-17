/**
 * @file subscriber_node.cpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@terpmail.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/subscriber_node.hpp"

Subscriber::Subscriber()
  : Node("subscriber") {
     subscription_ = this->create_subscription<std_msgs::msg::String>(
       "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
}

void Subscriber::topic_callback(
  const std_msgs::msg::String::SharedPtr msg) const {
  RCLCPP_INFO(this->get_logger(), "Incoming messgae: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
