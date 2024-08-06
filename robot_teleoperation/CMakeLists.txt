cmake_minimum_required(VERSION 3.8)
project(robot_teleoperation)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(robot_teleoperation_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# MongoDB


find_package(PkgConfig REQUIRED)
pkg_check_modules(MONGOCXX REQUIRED libmongocxx)

include_directories(${MONGOCXX_INCLUDE_DIRS})
link_directories(${MONGOCXX_LIBRARY_DIRS})


add_executable(teleop_controller_node src/teleoperation_backend_node.cpp
        include/robot_teleoperation/teleoperation_backend_node.hpp
)

target_link_libraries(teleop_controller_node ${MONGOCXX_LIBRARIES})


add_executable(direct_control_node src/direct_control_node.cpp
        include/robot_teleoperation/direct_control_node.hpp
)

add_executable(xbox_controller src/xboxController.cpp
        include/robot_teleoperation/xboxController.hpp
)

add_executable(space_mouse_node src/space_mouse_node.cpp
        include/robot_teleoperation/space_mouse_node.hpp
)

ament_target_dependencies(xbox_controller
        rclcpp
        geometry_msgs
        robot_teleoperation_interface
        sensor_msgs
        moveit_core
        moveit_ros_planning_interface
        moveit_ros_planning
        
        moveit_servo
        control_msgs
        rcl_interfaces
)

ament_target_dependencies(space_mouse_node
        rclcpp
        geometry_msgs
        robot_teleoperation_interface
        sensor_msgs
        moveit_core
        moveit_ros_planning_interface
        moveit_ros_planning
        moveit_servo
        control_msgs

)

ament_target_dependencies(teleop_controller_node
        "rclcpp"
        "moveit_ros_planning_interface"
        "moveit_core"
        "moveit_ros_planning"
        "robot_teleoperation_interface"
        "control_msgs"
)

ament_target_dependencies(direct_control_node
        "rclcpp"
        "moveit_ros_planning_interface"
        "moveit_core"
        "moveit_ros_planning"
        "robot_teleoperation_interface"
)

target_include_directories(space_mouse_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(teleop_controller_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(direct_control_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(xbox_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(space_mouse_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(teleop_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(direct_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(xbox_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS teleop_controller_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS direct_control_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS xbox_controller
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS space_mouse_node
        DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

