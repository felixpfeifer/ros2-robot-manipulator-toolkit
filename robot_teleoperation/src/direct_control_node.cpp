/******************************************************************************
 * Filename:    direct_movement_node.cpp
 * Description: Frontend node for the teleoperation interface to control the robot directly with the terminal
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     1.3
 * License:     MIT License
 ******************************************************************************/

#include "../include/robot_teleoperation/direct_control_node.hpp"

void setup() {
    // Set up the service clients
    align_tcp_client = node->create_client<robot_teleoperation_interface::srv::AllignTCP>("allign_tcp");
    select_tool_client = node->create_client<robot_teleoperation_interface::srv::SelectTool>("select_tool");
    move_robot_client = node->create_client<robot_teleoperation_interface::srv::MoveRobot>("move_robot");
    teach_point_client = node->create_client<robot_teleoperation_interface::srv::TeachPoint>("teach_point");
    tool_client = node->create_client<robot_teleoperation_interface::srv::Tool>("tool");
    move_point_client = node->create_client<robot_teleoperation_interface::srv::MovePoint>("move_point");
}

std::vector<std::string> getTerminalInput() {
    std::string input;
    std::vector<std::string> input_vector;
    std::getline(std::cin, input);
    std::istringstream iss(input);
    for (std::string input; iss >> input;) {
        input_vector.push_back(input);
    }
    return input_vector;
}

void controllLoop() {

    // Print the Message for the User
    RCLCPP_INFO(logger, "Welcome to the Direct Control Interface");
    RCLCPP_INFO(logger, "Commands: ");
    RCLCPP_INFO(logger, "align");
    RCLCPP_INFO(logger, "select <tool_id>");
    RCLCPP_INFO(logger, "teach <name>");
    RCLCPP_INFO(logger, "tool <open/close>");
    RCLCPP_INFO(logger, "move <joint/world/tool> <values>");
    RCLCPP_INFO(logger, "point <name>");
    RCLCPP_INFO(logger, "exit");

    while (rclcpp::ok()) {

        std::vector<std::string> input = getTerminalInput();
        if (input.size() == 0) {
            continue;
        }
        if (input[0] == "exit") {
            break;
        }
        // Align TCP
        if (input[0] == "align") {
            auto request = std::make_shared<robot_teleoperation_interface::srv::AllignTCP::Request>();
            request->hs_pose = false;
            auto result = align_tcp_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "TCP alignment was successful");
            } else {
                RCLCPP_ERROR(logger, "TCP alignment failed");
            }
        }
        // Select Tool
        else if (input[0] == "select") {
            auto request = std::make_shared<robot_teleoperation_interface::srv::SelectTool::Request>();
            request->tcp_id = std::stoi(input[1]);
            auto result = select_tool_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "Tool selection was successful");
                //std::string message = result.share().get()->message;
                //RCLCPP_INFO(logger, "%s", message.c_str());
            } else {
                RCLCPP_ERROR(logger, "Tool selection failed");
            }

        }

        // Teach current Point
        else if (input[0] == "teach") {
            auto request = std::make_shared<robot_teleoperation_interface::srv::TeachPoint::Request>();
            // Check if the name is given
            if (input.size() == 1) {
                RCLCPP_ERROR(logger, "Please provide a name for the point");
                continue;
            }
            request->name = input[1];
            auto result = teach_point_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "Teaching point was successful");
            } else {
                RCLCPP_ERROR(logger, "Teaching point failed");
            }
        }

        // Tool open Close
        else if (input[0] == "tool") {
            auto request = std::make_shared<robot_teleoperation_interface::srv::Tool::Request>();
            // Input is open or close
            if (input.size() == 1) {
                RCLCPP_ERROR(logger, "Please provide a state for the tool (open or close)");
                continue;
            }
            if (input[1] == "open") {
                request->close = false;
            } else if (input[1] == "close") {
                request->close = true;
            } else {
                RCLCPP_ERROR(logger, "Please provide a state for the tool (open or close)");
                continue;
            }

            auto result = tool_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "Gripper command was successful");
            } else {
                RCLCPP_ERROR(logger, "Gripper command was failed");
            }
        }

        // Move Robot
        else if (input[0] == "move") {
            auto request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
            // Second argument is the frame joint or world
            if (input.size() == 1) {
                RCLCPP_ERROR(logger, "Please provide a frame for the point (joint or world)");
                continue;
            }
            if (input[1] == "joint") {
                request->frame_id = 0;
                // After that a map of the joint number and the joint value
                // Check how many joints are given
                if (input.size() == 2) {
                    RCLCPP_ERROR(logger, "Please provide the joint values");
                    continue;
                }
                // Pair with <J1 0.0> <J2 0.0> ...
                for (int i = 2; i < input.size(); i += 2) {
                    std::string joint = input[i];
                    double value = stod(input[i + 1]);
                    // Convert the Joint Values to rad
                    value = value * M_PI / 180;
                    // Get the number of the Joint from the joint String
                    int joint_number = stoi(joint) - 1;
                    request->axis.push_back(joint_number);
                    request->data.push_back(value);
                }
            } else if (input[1] == "world") {
                request->frame_id = 1;
                // After that a map of the joint number and the joint value
                // Check how many Axis are given
                if (input.size() == 3) {
                    RCLCPP_ERROR(logger, "Please provide the pose values");
                    continue;
                }
                // Pair with <X 0.0> <Y 0.0> <Z 0.0> <QX 0.0> <QY 0.0> <QZ 0.0> <QW 0.0>
                for (int i = 2; i < input.size(); i += 2) {
                    std::string axis = input[i];
                    double value = stod(input[i + 1]);
                    // Convet the QX,QZ,QW to rad and XYZ to m from mm
                    // Get the order of the axis as id lower and upper case
                    if (axis == "X" || axis == "Y" || axis == "Z" || axis == "x" || axis == "y" || axis == "z") {
                        value = value / 1000;
                    } else if (axis == "RX" || axis == "RY" || axis == "RZ" || axis == "rx" || axis == "ry" ||
                               axis == "rz") {
                        value = value * M_PI / 180;
                    }

                    if (axis == "X" || axis == "x") {
                        request->axis.push_back(0);
                    } else if (axis == "Y" || axis == "y") {
                        request->axis.push_back(1);
                    } else if (axis == "Z" || axis == "z") {
                        request->axis.push_back(2);
                    } else if (axis == "RX" || axis == "rx") {
                        request->axis.push_back(3);
                    } else if (axis == "RY" || axis == "ry") {
                        request->axis.push_back(4);
                    } else if (axis == "RZ" || axis == "rz") {
                        request->axis.push_back(5);
                    }
                    request->data.push_back(value);
                }

            }else if( input[1] == "tool") {
                request->frame_id = 2;
                // After that a map of the joint number and the joint value
                // Check how many Axis are given
                if (input.size() == 3) {
                    RCLCPP_ERROR(logger, "Please provide the pose values");
                    continue;
                }
                // Pair with <X 0.0> <Y 0.0> <Z 0.0> <QX 0.0> <QY 0.0> <QZ 0.0> <QW 0.0>
                for (int i = 2; i < input.size(); i += 2) {
                    std::string axis = input[i];
                    double value = stod(input[i + 1]);
                    // Convet the QX,QZ,QW to rad and XYZ to m from mm
                    // Get the order of the axis as id lower and upper case
                    if (axis == "X" || axis == "Y" || axis == "Z" || axis == "x" || axis == "y" || axis == "z") {
                        value = value / 1000;
                    }else if (axis == "RX" || axis == "RY" || axis == "RZ" || axis == "rx" || axis == "ry" ||
                               axis == "rz") {
                        value = value * M_PI / 180;
                    }

                    if (axis == "X" || axis == "x") {
                        request->axis.push_back(0);
                    } else if (axis == "Y" || axis == "y") {
                        request->axis.push_back(1);
                    } else if (axis == "Z" || axis == "z") {
                        request->axis.push_back(2);
                    } else if (axis == "RX" || axis == "rx") {
                        request->axis.push_back(3);
                    } else if (axis == "RY" || axis == "ry") {
                        request->axis.push_back(4);
                    } else if (axis == "RZ" || axis == "rz") {
                        request->axis.push_back(5);
                    }
                    request->data.push_back(value);
                }
            }
            else {
                RCLCPP_ERROR(logger, "Please provide a frame for the point (joint, world or tool)");
                continue;
            }
            // Send the request
            auto result = move_robot_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "Moving robot was successful");
            } else {
                RCLCPP_ERROR(logger, "Moving robot failed");
            }
        } else if (input[0] == "point") {

            // Check if Point name is given
            if (input.size() == 1) {
                RCLCPP_ERROR(logger, "Please provide a name for the point");
                continue;
            }
            auto request = std::make_shared<robot_teleoperation_interface::srv::MovePoint::Request>();
            request->point_name = input[1];
            auto result = move_point_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node, result);
            if (result.share().get()->success) {
                RCLCPP_INFO(logger, "Moving to point was successful");
            } else {
                RCLCPP_ERROR(logger, "Moving to point failed");
            }
        } else {
            RCLCPP_ERROR(logger, "Unknown command");
        }
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("direct_movement_node");
    setup();
    controllLoop();
    rclcpp::shutdown();
    return 0;
}