/**
 * @file parameter_helper.cpp
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

#include "../include/parameter_helper.hpp"


ParameterHelper::ParameterHelper(): Node("parameter_helper") {
    // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    // param_desc.description = "This parameter is mine!";
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParameterHelper::timer_callback, this));
  }

void ParameterHelper::timer_callback() {
    std::string my_param =
    this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterHelper>());
  rclcpp::shutdown();
  return 0;
}
