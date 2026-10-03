cmake_minimum_required(VERSION 3.8)
project(manipulator_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

add_library(kinematics
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/kinematic_model.cpp
  src/dh_model.cpp
  src/urdf_model.cpp
)

add_executable(test_fk
  src/test_fk.cpp
)

add_executable(test_jacobian
  src/test_jacobian.cpp
)

add_executable(test_pseudoinverse
  src/test_pseudoinverse.cpp
)

add_executable(test_inverse_kinematics
  src/test_inverse_kinematics.cpp
)

add_executable(test_kinematic_model
  src/test_kinematic_model.cpp
)

add_executable(test_urdf_loading
  src/test_urdf_loading.cpp
)

add_executable(test_urdf_model
  src/test_urdf_model.cpp
)

add_executable(test_urdf_equivalence
  src/test_urdf_equivalence.cpp
)

target_include_directories(kinematics PUBLIC include)

target_link_libraries(test_fk kinematics)
target_link_libraries(test_jacobian kinematics)
target_link_libraries(test_pseudoinverse kinematics)
target_link_libraries(test_inverse_kinematics kinematics)
target_link_libraries(test_kinematic_model kinematics)
target_link_libraries(test_urdf_loading urdfdom::urdf_parser)
target_link_libraries(test_urdf_model kinematics)
target_link_libraries(test_urdf_equivalence kinematics)

target_link_libraries(kinematics
  PUBLIC Eigen3::Eigen
  PRIVATE urdfdom::urdf_parser
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
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
