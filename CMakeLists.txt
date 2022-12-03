cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# package building
find_package(ament_cmake REQUIRED)
# ros
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# services
find_package(rosidl_default_generators REQUIRED)

# add the msg's, and srv's
rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/<name>.msg"
  "srv/AddTwoInts.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # skips linter
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # skip other
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Runing lint tests in package.xml skipping above mentioned
  ament_lint_auto_find_test_dependencies()

  # set all the test files
  set(TESTFILES
    test/main.cpp
    test/basic_test.cpp
  )

  # add gtests
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(
    test_${PROJECT_NAME}
    rclcpp
    rclcpp_action
    std_msgs
  )

  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_${PROJECT_NAME} "${cpp_typesupport_target}")

  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

endif()

# add executables
add_executable(publisher_node src/publisher_node.cpp include/publisher_node.hpp)
ament_target_dependencies(
  publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(publisher_node "${cpp_typesupport_target}")

add_executable(subscriber_node src/subscriber_node.cpp include/subscriber_node.hpp)
ament_target_dependencies(
  subscriber_node
  rclcpp
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(subscriber_node "${cpp_typesupport_target}")

add_executable(client_node src/client_node.cpp)
ament_target_dependencies(
  client_node
  rclcpp
  std_msgs
  example_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(client_node "${cpp_typesupport_target}")


add_executable(param_helper_node src/parameter_helper.cpp)
ament_target_dependencies(
  param_helper_node
  rclcpp
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(param_helper_node "${cpp_typesupport_target}")

install(TARGETS
  publisher_node
  subscriber_node
  client_node
  param_helper_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ament_export_dependencies(rosidl_default_runtime)

ament_package()