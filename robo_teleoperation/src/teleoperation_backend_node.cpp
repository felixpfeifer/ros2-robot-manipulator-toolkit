/******************************************************************************
 * Filename:    teleoperation_backend_node.cpp
 * Description: Backend node for the teleoperation interface
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     2.0
 * License:     MIT License
 ******************************************************************************/

#include "robo_teleoperation/teleoperation_backend_node.hpp"


namespace robo_teleoperation {
    using namespace std::chrono_literals;

    TeleoperationBackendNode::TeleoperationBackendNode() : rclcpp::Node("TeleOpBackendNode") {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node = rclcpp::Node::make_shared("move_group_node", node_options);

        executor_.add_node(node);
        std::thread([&]() { executor_.spin(); }).detach();

        moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

        // Create a Subscriber for the Teleoperation Interfaces and call the Callback Function moveRobot
        teleoperation_interface_subscription = this->create_subscription<robot_teleoperation_interface::msg::Teleop>(
                "teleop_command_jog", 10, std::bind(&TeleoperationBackendNode::moveRobot, this, std::placeholders::_1));

        // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
        moveGroupInterface->setPlannerId("RRTConnect");

        // Move to Home Position
        moveNamedPositon("home");
        homing = false;

        // Set Joint Constraints to avoid flipping
        setJointConstraints();

        // Print Current Planning Frame
        RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", moveGroupInterface->getPlanningFrame().c_str());
        setJointConstraints();

        // Teleop Controller is ready to receive commands
        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is ready to receive commands");

        // Timer that runs every 20ms to get the current state of the robot
        timer_ = this->create_wall_timer(1s, std::bind(&TeleoperationBackendNode::timer_callback, this));

    }


    TeleoperationBackendNode::~TeleoperationBackendNode() {

    }

    void TeleoperationBackendNode::timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer Callback()");

        // Pose for the Home Position
        geometry_msgs::msg::PoseStamped home_pose;

        // Get the robot state
        moveit::core::RobotStatePtr robot_state_ptr = moveGroupInterface->getCurrentState();

        // Get the current Joint Position
        const auto joint_model = robot_state_ptr->getJointModelGroup(PLANNING_GROUP);
        std::vector<double> joint_values;
        robot_state_ptr->copyJointGroupPositions(joint_model, joint_values);

        // At the Start
        // Move the Robot to Home Position
        if (homing) {
            moveNamedPositon("home");
            homing = false;

            // Swtich the End Effector to the Camera
            switchEndEffector(false);
            // Align the TCP to the World Orientation
            alignTCP();
            // Swtich the End Effector to the Gripper
            //switchEndEffector(true);

        }
            // Else move the robot to a random Position of the Tool Frame
        else {
            moveRandom();
            homing = true;

        }
    }

    void TeleoperationBackendNode::moveNamedPositon(const std::string &position) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to Home Position");
        moveGroupInterface->setNamedTarget(position);
        moveGroupInterface->move();
    }

    void TeleoperationBackendNode::moveJoint(std::vector<double> joint_values, bool jog = false) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to Joint Position");
        if (jog) {
            // Get the Current Postion of the Joints and add to joint_values
            std::vector<double> current_joint_values = moveGroupInterface->getCurrentJointValues();
            for (int i = 0; i < joint_values.size(); ++i) {
                joint_values[i] += current_joint_values[i];
            }
        }
        moveGroupInterface->setJointValueTarget(joint_values);
        moveGroupInterface->move();
    }

    void TeleoperationBackendNode::moveWorldPose(geometry_msgs::msg::Pose pose) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to World Pose");
        moveGroupInterface->setPoseTarget(pose);
        moveGroupInterface->move();

    }

    void TeleoperationBackendNode::moveTCPPose(geometry_msgs::msg::Pose pose) {
        // Moves the Robot in the Coordinate System of the End Effector Link
        RCLCPP_INFO(this->get_logger(), "Move Robot to TCP Pose");
        // Get the current Pose of the End Effector
        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        // Set the Current Pose as the Start Pose
        moveGroupInterface->setStartStateToCurrentState();
        // Set the Target Pose to the given Pose
        moveGroupInterface->setPoseTarget(pose);
        moveGroupInterface->move();


    }

    void TeleoperationBackendNode::moveRandom() {
        RCLCPP_INFO(this->get_logger(), "Move Robot to Random z Position");

        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        // Get the current Set the Current Pose as the Start Pose
        moveGroupInterface->setStartStateToCurrentState();
        // Set the Target Pose to a random Position from the current Position
        // Each Axis is moved by a random value between -0.1 and 0.1 one for each Axis
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x += 0.1 * ((double) rand() / RAND_MAX - 0.5);
        target_pose.position.y += 0.1 * ((double) rand() / RAND_MAX - 0.5);
        target_pose.position.z += 0.1 * ((double) rand() / RAND_MAX - 0.5);

        // Set the Orientation to the current Orientation
        target_pose.orientation = current_pose.pose.orientation;
        // Now the Robot should move to the random Orientation in the range of +- 20 degrees
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disA(-M_PI / 9, M_PI / 9);
        std::uniform_real_distribution<> disB(-M_PI / 9, M_PI / 9);
        std::uniform_real_distribution<> disC(-M_PI / 9, M_PI / 9);
        target_pose = changeOrientation(target_pose, disA(gen), disB(gen), disC(gen));

        // Call the movePose with the new Random Pose
        moveWorldPose(target_pose);

    }

    void TeleoperationBackendNode::setJointConstraints() {
        // Add joint constraints to avoid flipping
        moveit_msgs::msg::Constraints joint_constraints;

        // Constraint for joint 1 to avoid flipping
        moveit_msgs::msg::JointConstraint joint1_constraint;
        joint1_constraint.joint_name = "joint_1";
        joint1_constraint.position = moveGroupInterface->getCurrentJointValues()[0];
        // Limit the joint to 90 degrees above and below the current position
        joint1_constraint.tolerance_above = M_PI / 2;  // Adjust as necessary
        joint1_constraint.tolerance_below = M_PI / 2;  // Adjust as necessary

        joint1_constraint.weight = 1.0;
        joint_constraints.joint_constraints.push_back(joint1_constraint);

        // Constraint for joint 4
        moveit_msgs::msg::JointConstraint joint4_constraint;
        joint4_constraint.joint_name = "joint_4";
        joint4_constraint.position = moveGroupInterface->getCurrentJointValues()[3];
        // Limit the joint to 90 degrees above and below the current position
        joint4_constraint.tolerance_above = M_PI / 2;  // Adjust as necessary
        joint4_constraint.tolerance_below = M_PI / 2;  // Adjust as necessary
        joint4_constraint.weight = 1.0;
        joint_constraints.joint_constraints.push_back(joint4_constraint);

        moveGroupInterface->setPathConstraints(joint_constraints);

    }

    void TeleoperationBackendNode::moveRobot(robot_teleoperation_interface::msg::Teleop::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Move Robot");
        // Get the Frame in which the Robot should move
        int frame = msg->frame;
        // Print out the msg for debugging
        for (int i = 0; i < msg->axis_id.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Axis ID: %d, Value: %f", msg->axis_id[i], msg->axis_values[i]);
        }
        switch (frame) {
            case 0: {
                // Move the Robot in the Joint Frame
                // Create a zero initialized JointData with 6 Joints with old values
                std::vector<double> joint_values(6, 0.0);
                // Set the vector to the old values
                joint_values = moveGroupInterface->getCurrentJointValues();
                // Find in the tokes first the axis and then the value
                for (int i = 0; i < msg->axis_id.size(); ++i) {
                    int axis = msg->axis_id[i];
                    double value = msg->axis_values[i];
                    joint_values[axis] = value;
                }
                // Print out the Joint Values
                for (int i = 0; i < joint_values.size(); ++i) {
                    RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, joint_values[i]);
                }
                moveJoint(joint_values, false);
                break;
            }
            case 1: {
                // Move te Robot in the World Frame
                // Get the Pose of the End Effector in the World Coordinate System
                geometry_msgs::msg::Pose pose = moveGroupInterface->getCurrentPose().pose;
                // Add the values of the array to the current Pose
                for (int i = 0; i < msg->axis_id.size(); ++i) {
                    int id = msg->axis_id[i];
                    switch (id) {
                        case 0:
                            pose.position.x += msg->axis_values[i];
                            break;
                        case 1:
                            pose.position.y += msg->axis_values[i];
                            break;
                        case 2:
                            pose.position.z += msg->axis_values[i];
                            break;
                        case 3:
                            changeOrientation(pose, msg->axis_values[i], 0);
                            break;
                        case 4:
                            changeOrientation(pose, msg->axis_values[i], 1);
                            break;
                        case 5:
                            changeOrientation(pose, msg->axis_values[i], 2);
                            break;
                        default:
                            RCLCPP_ERROR(this->get_logger(), "Invalid Axis ID");
                    }
                }
                moveWorldPose(pose);
                break;
            }
            case 2:
                // Move the Robot in the Tool Frame
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid Frame");

        }


    }

    void TeleoperationBackendNode::changeOrientation(geometry_msgs::msg::Pose &pose, double angle, int axis) {
        tf2::Quaternion q;
        tf2::convert(pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        switch (axis) {
            case 0:
                roll += angle;
                break;
            case 1:
                pitch += angle;
                break;
            case 2:
                yaw += angle;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid Axis");
        }
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q, pose.orientation);
    }

    void TeleoperationBackendNode::switchEndEffector(bool gripper) {
        if (gripper) {
            moveGroupInterface->setEndEffectorLink(GRIPPER_LINK);
        } else {
            moveGroupInterface->setEndEffectorLink(CAMERA_LINK);

        }


    }

    void TeleoperationBackendNode::alignTCP() {
        RCLCPP_INFO(this->get_logger(), "Aligning TCP to World Orientation");

        // Get the current pose of the end effector
        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();

        // Convert current orientation to a quaternion
        tf2::Quaternion q;
        tf2::convert(current_pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Convert the orientation angels from rad to degree
        roll = roll * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        yaw = yaw * 180 / M_PI;


        RCLCPP_INFO(this->get_logger(), "Current Orientation: roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
        // Calculate the new orientation with each angle is a multiple of 90 degrees
        roll = round(roll / 90) * 90;
        pitch = round(pitch / 90) * 90;
        yaw = round(yaw / 90) * 90;

        // Convert the orientation angels from degree to rad
        roll = roll * M_PI / 180;
        pitch = pitch * M_PI / 180;
        yaw = yaw * M_PI / 180;

        // Move the robot to the new orientation
        moveWorldPose(current_pose.pose);

    }

    geometry_msgs::msg::Pose &
    TeleoperationBackendNode::changeOrientation(geometry_msgs::msg::Pose &target_pose, double roll, double pitch,
                                                double yaw) {
        tf2::Quaternion q;
        tf2::convert(target_pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll_, pitch_, yaw_;
        m.getRPY(roll_, pitch_, yaw_);
        yaw_ += yaw;
        pitch_ += pitch;
        roll_ += roll;
        q.setRPY(roll_, pitch_, yaw_);
        tf2::convert(q, target_pose.orientation);
        return target_pose;
    }

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<robo_teleoperation::TeleoperationBackendNode>());
    rclcpp::shutdown();
    return 0;
}
