//
// Created by felix on 18.06.24.
//

#ifndef ROBO_TELEOPERATION_DIRECT_CONTROL_NODE_HPP
#define ROBO_TELEOPERATION_DIRECT_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

// Services
#include "robot_teleoperation_interface/srv/allign_tcp.hpp"
#include "robot_teleoperation_interface/srv/select_tool.hpp"
#include "robot_teleoperation_interface/srv/move_robot.hpp"
#include "robot_teleoperation_interface/srv/hand2_eye.hpp"
#include "robot_teleoperation_interface/srv/teach_point.hpp"
#include "robot_teleoperation_interface/srv/tool.hpp"
#include "robot_teleoperation_interface/srv/move_point.hpp"

// Logger
rclcpp::Logger logger = rclcpp::get_logger("direct_control_node");

// Service Clients
rclcpp::Client<robot_teleoperation_interface::srv::AllignTCP>::SharedPtr align_tcp_client;
rclcpp::Client<robot_teleoperation_interface::srv::SelectTool>::SharedPtr select_tool_client;
rclcpp::Client<robot_teleoperation_interface::srv::MoveRobot>::SharedPtr move_robot_client;
rclcpp::Client<robot_teleoperation_interface::srv::Hand2Eye>::SharedPtr hand2_eye_client;
rclcpp::Client<robot_teleoperation_interface::srv::TeachPoint>::SharedPtr teach_point_client;
rclcpp::Client<robot_teleoperation_interface::srv::Tool>::SharedPtr tool_client;
rclcpp::Client<robot_teleoperation_interface::srv::MovePoint>::SharedPtr move_point_client;

rclcpp::Node::SharedPtr node;

void setup();

std::vector<std::string> getTerminalInput();

// Function to control the robot directly
void controllLoop();

// Main function
int main(int argc, char *argv[]);


#endif //ROBO_TELEOPERATION_DIRECT_CONTROL_NODE_HPP
