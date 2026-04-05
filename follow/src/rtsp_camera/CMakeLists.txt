cmake_minimum_required(VERSION 3.8)
project(rtsp_camera)

# ================= Compiler options =================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= Dependencies =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# ================= Build node =================
add_executable(rtsp_camera_node
  RtspCamera.cpp
)

target_include_directories(rtsp_camera_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(rtsp_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  camera_info_manager
)

target_link_libraries(rtsp_camera_node
  ${OpenCV_LIBS}
)

# ================= Install =================
install(TARGETS rtsp_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================= Testing =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
