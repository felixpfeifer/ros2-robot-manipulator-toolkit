/******************************************************************************
 * Filename:    direct_movement_node.cpp
 * Description: Frontend node for the teleoperation interface to control the robot directly with the terminal
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     1.2
 * License:     MIT License
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "robot_teleoperation_interface/msg/teleop.hpp"

rclcpp::Logger logger = rclcpp::get_logger("direct_movement_node");

void controllLoop(rclcpp::Node::SharedPtr node) {

    RCLCPP_INFO(logger, "Publishing Message");
    //rclcpp::init(0, nullptr);
    //auto node = std::make_shared<rclcpp::Node>("direct_movement_node");
    auto teleop_publisher = node->create_publisher<robot_teleoperation_interface::msg::Teleop>("teleop_command_jog", 10);


    while(rclcpp::ok()) {
        RCLCPP_INFO(logger, "Enter Command: ");
        std::string command;
        std::getline(std::cin, command);
        // Parse the Line
        std::istringstream iss(command);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
                  std::back_inserter(tokens));
        if (tokens.size() == 0) {
            RCLCPP_INFO(logger, "No Command entered");
            continue;
        }
        // Check for the Command exit
        if (tokens[0] == "exit") {
            RCLCPP_INFO(logger, "Exit Command received");
            break;
        }
        // Check for the Command move
        // Enter moveJoint to move one or mulptiple joints
        // Format for each joint: moveJoint <joint_number> <value>
        if (tokens[0] == "joint") {
            if (tokens.size() < 3) {
                RCLCPP_INFO(logger, "Not enough arguments for moveJoint");
                continue;
            }
            robot_teleoperation_interface::msg::Teleop msg;
            msg.frame = 0;
            // Create a zero initialized JointData with 6 Joints
            // Find in the tokes first the axis and then the value
            for (int i = 1; i < tokens.size(); i += 2) {
                int axis = std::stoi(tokens[i]);
                double value = std::stod(tokens[i + 1]);
                msg.axis_id.push_back(axis);
                msg.axis_values.push_back(value);
            }
            teleop_publisher->publish(msg);
        }
        // Enter moveWorldPose to move the robot to a specific position in the world frame only one position is needed others ore optional
        // Format: moveWorldPose <x> <y> <z> <a> <b> <c>
        // Every Axis is given with a char and a double as a pair
        if (tokens[0] == "world") {
            if (tokens.size() < 3) {
                RCLCPP_INFO(logger, "Not enough arguments for moveWorldPose");
                continue;
            }
            robot_teleoperation_interface::msg::Teleop msg;
            msg.frame = 1;
            for (int i = 1; i < tokens.size(); i += 2) {
                // Axis id is a number from 0 to 5 for the 6 axis
                char axis = tokens[i][0];
                switch (axis) {
                    case 'x':
                        axis = 0;
                        break;
                    case 'y':
                        axis = 1;
                        break;
                    case 'z':
                        axis = 2;
                        break;
                    case 'a':
                        axis = 3;
                        break;
                    case 'b':
                        axis = 4;
                        break;
                    case 'c':
                        axis = 5;
                        break;
                    default:
                        RCLCPP_INFO(logger, "Unknown Axis");
                        continue;
                }

                double value = std::stod(tokens[i + 1]);
                msg.axis_id.push_back(axis);
                msg.axis_values.push_back(value);
            }
            teleop_publisher->publish(msg);
        }
    }

    rclcpp::shutdown();

}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("direct_movement_node");
    controllLoop(node);
    rclcpp::shutdown();
    return 0;
}