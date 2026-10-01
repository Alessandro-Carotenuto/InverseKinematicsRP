cmake_minimum_required(VERSION 3.8)
project(manipulator_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(kinematics
  src/forward_kinematics.cpp
)

add_executable(test_fk
  src/test_fk.cpp
)

add_executable(test_jacobian
  src/test_jacobian.cpp
)

target_include_directories(kinematics PUBLIC
  include
)

target_link_libraries(test_fk
  kinematics
)

target_link_libraries(test_jacobian
  kinematics
)

target_link_libraries(kinematics
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
