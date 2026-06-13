cmake_minimum_required(VERSION 3.8)
project(rv2_csm_topic_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)
find_package(rv2_control_signal_transport REQUIRED)

# ── topic_bridge (composable node) ────────────────────────────────────────────
add_library(topic_bridge_component SHARED
  src/topic_bridge_node.cpp
)
target_include_directories(topic_bridge_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(topic_bridge_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  rv2_interfaces
  rv2_control_signal_transport
)
rclcpp_components_register_node(topic_bridge_component
  PLUGIN "rv2_csm_topic_bridge::TopicBridgeNode"
  EXECUTABLE topic_bridge
)
install(TARGETS topic_bridge_component topic_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── launch & config ───────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Scenario tests for TopicBridgeNode.
  # The test includes topic_bridge_node.cpp directly so it can construct the
  # node in-process without loading a plugin.  The control_signal_transport
  # shared library is linked to provide the ControlSignalFactory registry and
  # ControlSignalManager implementation.
  ament_add_gtest(test_topic_bridge
    test/test_topic_bridge.cpp
  )
  target_include_directories(test_topic_bridge PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_topic_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    std_msgs
    rv2_interfaces
    rv2_control_signal_transport
  )
  # Link the transport shared lib (singleton factory registry + CSM impl).
  # The test does NOT link topic_bridge_component to avoid duplicate symbol
  # issues: instead it includes the .cpp directly (see test file).
  target_link_libraries(test_topic_bridge
    rv2_control_signal_transport::control_signal_transport
  )
endif()

ament_package()
