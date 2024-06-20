cmake_minimum_required(VERSION 3.8)
project(robo_teleoperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(robot_teleoperation_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# MongoDB

find_package(PkgConfig REQUIRED)
pkg_check_modules(MONGOCXX REQUIRED libmongocxx)

include_directories(${MONGOCXX_INCLUDE_DIRS})
link_directories(${MONGOCXX_LIBRARY_DIRS})


add_executable(teleop_controller_node src/teleoperation_backend_node.cpp
        include/robo_teleoperation/teleoperation_backend_node.hpp
)

target_link_libraries(teleop_controller_node ${MONGOCXX_LIBRARIES})



add_executable(direct_control_node src/direct_control_node.cpp
        include/robo_teleoperation/direct_control_node.hpp
)



ament_target_dependencies(teleop_controller_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_core"
  "moveit_ros_planning"
  "robot_teleoperation_interface"
)

ament_target_dependencies(direct_control_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_core"
  "moveit_ros_planning"
  "robot_teleoperation_interface"
)



target_include_directories(teleop_controller_node  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(direct_control_node  PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_compile_features(teleop_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(direct_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS teleop_controller_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS direct_control_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
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