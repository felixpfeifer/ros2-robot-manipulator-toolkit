//
// Created by felix on 05.06.24.
//

#ifndef ROBO_TELEOPERATION_KEYBORD_TELEOP_CONTROLLER_HPP
#define ROBO_TELEOPERATION_KEYBORD_TELEOP_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>


#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>  // Add this header for F_SETFL and O_NONBLOCK
#include <sys/select.h>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/node.hpp>

#include "robot_teleoperation_interface/msg/teleop.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <map>
namespace robo_teleoperation {

    class Keybord_Teleop_Controller : public rclcpp::Node {
    public:
        Keybord_Teleop_Controller();

        ~Keybord_Teleop_Controller();

        void controllLoop();



    private:

        std::vector<double> joint_values;

        // TODO: Create a Publisher for the Teleoperation Backend to communicate with the Frontend

        char getKey();
        int printSelectMenue();

        int mode = 0;
        rclcpp::Logger logger = get_logger();
        rclcpp::Node::SharedPtr node;

        // Robot Vars
        double speed;
        double speed_increment = 0.1;

        // Data Values for the axis
        std::vector<double> axis_values = {0, 0, 0, 0, 0, 0};
        double stepp = 1; // Step for the Joint Movement in deg or mm in cartesian space
        // Tool
        bool tool_open = false;


        // Publisch the Command with the robot_teleoperation_interface msgs
        rclcpp::Publisher<robot_teleoperation_interface::msg::Teleop>::SharedPtr teleop_publisher;

        // Map 12 Keys to the postive and negative direction of for the 6 Joints
        std::map<char, std::pair<int, int>> keyMap = {
                {'w', {1, 1}},
                {'s', {1, -1}},
                {'a', {2, 1}},
                {'d', {2, -1}},
                {'q', {3, 1}},
                {'e', {3, -1}},
                {'r', {4, 1}},
                {'f', {4, -1}},
                {'t', {5, 1}},
                {'g', {5, -1}},
                {'y', {6, 1}},
                {'h', {6, -1}}
        };
        // Optionls keys to swtich the movement Frame or the Speed of the Robot and to open close the Tool
        // 1. Joint Space 2. World Frame 3. Tool Frame 4./5. Speed increase and decrease 6. Open Close Tool
        std::map<char, int> optionMap = {
                {'1', 1},
                {'2', 2},
                {'3', 3},
                {'4', 4},
                {'5', 5},
                {'6', 6}
        };
    };



} // namespace robo_teleoperation
#endif //ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP