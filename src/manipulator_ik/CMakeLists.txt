cmake_minimum_required(VERSION 3.8)
project(manipulator_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(kinematics
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/kinematic_model.cpp
  src/dh_model.cpp
  src/urdf_model.cpp
)

add_executable(manipulator_ik_node
  src/manipulator_ik_node.cpp
)
ament_target_dependencies(manipulator_ik_node rclcpp rcl_interfaces std_srvs sensor_msgs)
target_link_libraries(manipulator_ik_node kinematics yaml-cpp::yaml-cpp)
target_include_directories(kinematics PUBLIC include)
target_compile_features(manipulator_ik_node PRIVATE cxx_std_17)

target_link_libraries(kinematics
  PUBLIC Eigen3::Eigen
  PRIVATE urdfdom::urdf_parser
          yaml-cpp::yaml-cpp
)

install(TARGETS manipulator_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY Robots
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
