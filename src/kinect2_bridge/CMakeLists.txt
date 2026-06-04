cmake_minimum_required(VERSION 3.8)
project(kinect2_bridge)

# freenect2 cmake config lives at /usr/local/lib/cmake/freenect2
set(freenect2_DIR /usr/local/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(kinect2_bridge_node src/kinect2_bridge_node.cpp)

target_compile_options(kinect2_bridge_node PRIVATE
  -Wall -Wextra
  -O3
)

target_include_directories(kinect2_bridge_node PRIVATE
  ${freenect2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(kinect2_bridge_node
  ${freenect2_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(kinect2_bridge_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS kinect2_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
