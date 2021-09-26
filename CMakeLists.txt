cmake_minimum_required(VERSION 3.8)
project(biped_ik_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(mx_joint_controller_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# Include hpp files
include_directories(
  include
  external
  ${EIGEN3_INCLUDE_DIR}
)

# Build biped_ik_simple library
add_library(biped_ik_simple_lib
  src/biped_ik_simple_node.cpp
  external/IKWalk/HumanoidModel.cpp
  external/IKWalk/LegIK.cpp)
ament_target_dependencies(biped_ik_simple_lib
  rclcpp
  biped_interfaces
  mx_joint_controller_msgs
  tf2
  Eigen3)

# Build biped_ik_simple executable
add_executable(biped_ik_simple
  src/biped_ik_simple_main.cpp)
target_link_libraries(biped_ik_simple
  biped_ik_simple_lib)

# Install
install(TARGETS
  biped_ik_simple
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS biped_ik_simple_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
