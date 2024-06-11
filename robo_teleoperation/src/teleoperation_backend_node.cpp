
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
        // Timer that runs every 20ms to get the current state of the robot
        //timer_ = this->create_wall_timer(1s, std::bind(&TeleoperationBackendNode::timer_callback, this));


        // Create a Subscriber for the Teleoperation Interfaces and call the Callback Function moveRobot
        teleoperation_interface_subscription = this->create_subscription<robot_teleoperation_interface::msg::Teleop>(
                "teleop_command_jog", 10, std::bind(&TeleoperationBackendNode::moveRobot, this, std::placeholders::_1));


        // Print Current Planning Frame
        RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", moveGroupInterface->getPlanningFrame().c_str());
        setJointConstraints();

        // Teleop Controller is ready to receive commands
        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is ready to receive commands");

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
        // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
        moveGroupInterface->setPlannerId("RRTConnect");
        // At the Start
        // Move the Robot to Home Position
        if (homing) {
            moveNamedPositon("home");
            homing = false;
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

    void TeleoperationBackendNode::moveJoint(std::vector<double> joint_values) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to Joint Position");
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
        tf2::Quaternion q;
        tf2::convert(target_pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw += disA(gen);
        pitch += disB(gen);
        roll += disC(gen);
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q, target_pose.orientation);

        moveGroupInterface->setPoseTarget(target_pose);
        moveGroupInterface->setPlanningTime(10);
        moveGroupInterface->setNumPlanningAttempts(10);
        moveGroupInterface->move();
    }

    void TeleoperationBackendNode::setJointConstraints() {
        // Add joint constraints to avoid flipping
        moveit_msgs::msg::Constraints joint_constraints;

        // Constraint for joint 0 (joint_1) to avoid flipping
        moveit_msgs::msg::JointConstraint joint0_constraint;
        joint0_constraint.joint_name = "joint_1";
        joint0_constraint.position = moveGroupInterface->getCurrentJointValues()[0];
        // Limit the joint to 90 degrees above and below the current position
        joint0_constraint.tolerance_above = M_PI / 2;  // Adjust as necessary
        joint0_constraint.tolerance_below = M_PI / 2;  // Adjust as necessary
        joint0_constraint.weight = 1.0;
        joint_constraints.joint_constraints.push_back(joint0_constraint);

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
        // Read the Speed
        //moveGroupInterface->setMaxVelocityScalingFactor(msg->speed);
        // Read to open close the gripper
        if (msg->gpios.size() > 0 && msg->gpios[0] == 1) {
            // Open the Gripper
            // TODO: Send GPIO Command with Publisher
        } else {
            // Close the Gripper
            // TODO: Send GPIO Command with Publisher
        }

        // check if a Named Position is given
        if (!msg->target.empty()) {
            moveNamedPositon(msg->target);
            return;
        }

        // Get the Frame # 0 for a Joint Pose, 1 for a Cartesian Pose in world, 2 for a Cartesian Pose the Tool Frame
        int frame = msg->frame;
        switch (frame) {
            case 0: {
                // TODO: Robot is not moveing
                // Calulate a new Positon of joints with the given values
                std::vector<double> joint_values = msg->data_values;
                for (int i = 0; i < joint_values.size(); i++) {
                    // data_values are the delta values for the joints and in grad
                    // convert to rad
                    joint_values[i] = joint_values[i] * M_PI / 180;
                    joint_values[i] = moveGroupInterface->getCurrentJointValues()[i] + joint_values[i];
                }
                moveJoint(joint_values);
                break;
            }
            case 1: {

                geometry_msgs::msg::Pose pose = getPose(msg->data_values, moveGroupInterface->getCurrentPose().pose);
                moveWorldPose(pose);
                break;
            }
            case 2: {
                geometry_msgs::msg::Pose pose = getPose(msg->data_values, moveGroupInterface->getCurrentPose().pose);
                moveTCPPose(pose);
                break;
            }
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown Frame");
                break;
        }
    }

    geometry_msgs::msg::Pose
    TeleoperationBackendNode::getPose(std::vector<double> value) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = value[0];
        pose.position.y = value[1];
        pose.position.z = value[2];
        tf2::Quaternion q;
        q.setRPY(value[3], value[4], value[5]);
        tf2::convert(q, pose.orientation);
        return pose;
    }

    geometry_msgs::msg::Pose
    TeleoperationBackendNode::getPose(std::vector<double> value, geometry_msgs::msg::Pose pose) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = pose.position.x + value[0];
        new_pose.position.y = pose.position.y + value[1];
        new_pose.position.z = pose.position.z + value[2];
        // Add to the Rotations in RPY the values of the array
        tf2::Quaternion q;
        tf2::convert(pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        roll += value[3];
        pitch += value[4];
        yaw += value[5];
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q, new_pose.orientation);
        return new_pose;

    }


}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<robo_teleoperation::TeleoperationBackendNode>());
    rclcpp::shutdown();
    return 0;
}
