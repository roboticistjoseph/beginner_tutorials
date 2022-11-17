/**
 * @file publisher_node.cpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@terpmail.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/publisher_node.hpp"

Publisher::Publisher()
    : Node("publisher"), count_(0) {
       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
       timer_ = this->create_wall_timer(500ms,
        std::bind(&Publisher::timer_callback, this));
}

void Publisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Developer- Joseph:  "
             + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
