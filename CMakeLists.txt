cmake_minimum_required(VERSION 3.8)
project(rv2_csm_topic_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(r1_interfaces REQUIRED)
find_package(rv2_control_signal_transport REQUIRED)
set(bridge_dependencies rclcpp rclcpp_components sensor_msgs geometry_msgs std_msgs
  r1_interfaces rv2_control_signal_transport)
add_library(topic_bridge_component SHARED src/topic_bridge_node.cpp)
target_include_directories(topic_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(topic_bridge_component ${bridge_dependencies})
target_link_libraries(topic_bridge_component
  rv2_control_signal_transport::r1_control_signal_transport)
rclcpp_components_register_node(topic_bridge_component
  PLUGIN "rv2_csm_topic_bridge::TopicBridgeNode"
  EXECUTABLE topic_bridge)
install(TARGETS topic_bridge_component topic_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${bridge_dependencies})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_config test/unit/test_bridge_config.cpp)
  target_link_libraries(test_bridge_config topic_bridge_component)
  ament_add_gtest(test_input_mailbox test/unit/test_input_mailbox.cpp)
  target_link_libraries(test_input_mailbox topic_bridge_component)
  ament_add_gtest(test_topic_bridge test/integration/test_topic_bridge.cpp TIMEOUT 120)
  target_link_libraries(test_topic_bridge topic_bridge_component)
  set_tests_properties(test_bridge_config test_input_mailbox PROPERTIES LABELS "unit")
  set_tests_properties(test_topic_bridge PROPERTIES LABELS "integration")
endif()
ament_package()
