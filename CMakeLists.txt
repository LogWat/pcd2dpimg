cmake_minimum_required(VERSION 3.8)
project(pcd2dpimg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(pcd2dpimg SHARED src/pcd2dpimg.cpp)
rclcpp_components_register_nodes(pcd2dpimg "pcd2dpimg::Pcd2DpImg")
target_compile_definitions(pcd2dpimg
  PRIVATE "PCD2DPIMG_BUILDING_LIBRARY"
)
target_include_directories(pcd2dpimg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pcd2dpimg PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  pcd2dpimg
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "rclcpp_components"
  "OpenCV"
  "cv_bridge"
)

install(TARGETS pcd2dpimg
  DESTINATION lib)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
