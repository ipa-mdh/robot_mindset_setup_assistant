cmake_minimum_required(VERSION 3.8)
project({{args.package.name}})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  pluginlib
  std_msgs
  control_msgs
  geometry_msgs
  tf2_geometry_msgs
  moveit_msgs
  std_srvs
  moveit_ros_planning
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#####################
## Component Nodes ##
#####################
# This node converts joystick input to servo commands
set(SERVO_CONTROLLER_INPUT servo_controller_input)
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "{{args.package.name}}::JoyToServoPub")

######################
## Executable Nodes ##
######################
# keyboard input node
set(SERVO_KEYBOARD_INPUT servo_keyboard_input)
add_executable(${SERVO_KEYBOARD_INPUT} src/servo_keyboard_input.cpp)
target_include_directories(${SERVO_KEYBOARD_INPUT} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${SERVO_KEYBOARD_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# servo test motions
set(SERVO_TEST_MOTIONS servo_test_motions)
add_executable(${SERVO_TEST_MOTIONS} src/servo_test_motions.cpp)
target_include_directories(${SERVO_TEST_MOTIONS} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${SERVO_TEST_MOTIONS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############

install(
  TARGETS
    ${SERVO_KEYBOARD_INPUT}
    ${SERVO_TEST_MOTIONS}
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  DESTINATION
    lib
)

install(DIRECTORY launch config example urdf
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
