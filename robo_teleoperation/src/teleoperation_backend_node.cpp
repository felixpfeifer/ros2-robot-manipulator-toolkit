/******************************************************************************
 * Filename:    teleoperation_backend_node.cpp
 * Description: Backend node for the teleoperation interface
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     3.0
 * License:     MIT License
 ******************************************************************************/

#include "robo_teleoperation/teleoperation_backend_node.hpp"


namespace robo_teleoperation {

    using namespace std::chrono_literals;

    TeleoperationBackendNode::TeleoperationBackendNode() : rclcpp::Node("TeleOpBackendNode") {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);


        node = rclcpp::Node::make_shared("move_group_node", node_options);

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        executor_.add_node(node);
        std::thread([&]() { executor_.spin(); }).detach();

        moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

        // Initialize the Services as Server
        move_robot_service = this->create_service<robot_teleoperation_interface::srv::MoveRobot>(
                "move_robot", std::bind(&TeleoperationBackendNode::moveRobotService, this, std::placeholders::_1,
                                        std::placeholders::_2));

        move_point_service = this->create_service<robot_teleoperation_interface::srv::MovePoint>(
                "move_point", std::bind(&TeleoperationBackendNode::movePointService, this, std::placeholders::_1,
                                        std::placeholders::_2));

        select_tool_service = this->create_service<robot_teleoperation_interface::srv::SelectTool>(
                "select_tool", std::bind(&TeleoperationBackendNode::selectToolService, this, std::placeholders::_1,
                                         std::placeholders::_2));
        allign_tcp_service = this->create_service<robot_teleoperation_interface::srv::AllignTCP>(
                "allign_tcp", std::bind(&TeleoperationBackendNode::allignTCPService, this, std::placeholders::_1,
                                        std::placeholders::_2));

        teach_point_service = this->create_service<robot_teleoperation_interface::srv::TeachPoint>(
                "teach_point", std::bind(&TeleoperationBackendNode::teachPointService, this, std::placeholders::_1,
                                         std::placeholders::_2));

        tool_service = this->create_service<robot_teleoperation_interface::srv::Tool>(
                "tool", std::bind(&TeleoperationBackendNode::toolService, this, std::placeholders::_1,
                                  std::placeholders::_2));

        // GPIO Publisher and Subscriber
        gpio_publisher = this->create_publisher<floatarray>("/gpio_controller/commands", 10);
        gpio_subscriber = this->create_subscription<CmdType>(
                "/gpio_controller/inputs", 10,
                std::bind(&TeleoperationBackendNode::gpioCallback, this, std::placeholders::_1));

        // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
        moveGroupInterface->setPlannerId("RRTConnect");

        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is starting");


        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

        speed_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("speed", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("position", 10);

        // Set Joint Constraints to avoid flipping
        setJointConstraints();

        gripper_open = true;

        // Print Current Planning Frame
        RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", moveGroupInterface->getPlanningFrame().c_str());

        // Teleop Controller is ready to receive commands
        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is ready to receive commands");

        // Timer that runs every 20ms to get the current state of the robot
        //timer_ = this->create_wall_timer(1s, std::bind(&TeleoperationBackendNode::timer_callback, this));
        timer_ = this->create_wall_timer(100ms, std::bind(&TeleoperationBackendNode::calculateSpeed_Timecallback, this));

        // Move to Home Position
        moveNamedPositon("home");
        homing = false;

        // At the Start the End Effector is the Gripper
        switchEndEffector(true);
    }


    TeleoperationBackendNode::~TeleoperationBackendNode() = default;

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
        auto result = moveGroupInterface->move();
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Move Successful");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move Failed");
            throw std::runtime_error("Move Failed");
        }
    }

    void TeleoperationBackendNode::moveWorldPose(geometry_msgs::msg::Pose pose) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to World Pose");
        moveGroupInterface->setStartState(*moveGroupInterface->getCurrentState());
        moveGroupInterface->setPoseTarget(pose);
        auto result = moveGroupInterface->move();
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Move Successful");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move Failed");
            throw std::runtime_error("Move Failed");
        }
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
        joint1_constraint.tolerance_above = M_PI / 2;// Adjust as necessary
        joint1_constraint.tolerance_below = M_PI / 2;// Adjust as necessary

        joint1_constraint.weight = 1.0;
        joint_constraints.joint_constraints.push_back(joint1_constraint);

        // Constraint for joint 4
        moveit_msgs::msg::JointConstraint joint4_constraint;
        joint4_constraint.joint_name = "joint_4";
        joint4_constraint.position = moveGroupInterface->getCurrentJointValues()[3];
        // Limit the joint to 90 degrees above and below the current position
        joint4_constraint.tolerance_above = M_PI / 2;// Adjust as necessary
        joint4_constraint.tolerance_below = M_PI / 2;// Adjust as necessary
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

        RCLCPP_INFO(this->get_logger(), "New Orientation: roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

        // Convert the orientation angels from degree to rad
        roll = roll * M_PI / 180;
        pitch = pitch * M_PI / 180;
        yaw = yaw * M_PI / 180;

        // Set the new orientation
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q, current_pose.pose.orientation);


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

    bool TeleoperationBackendNode::safePosetoMongoDB(geometry_msgs::msg::Pose pose, std::string name) {

        // Safe the Pose to the MongoDB
        RCLCPP_INFO(this->get_logger(), "Safe Pose to MongoDB");
        // Use the Poses Collection

        // Check if name is already in the Database
        auto filter = bsoncxx::builder::stream::document{} << "name" << name
                                                           << bsoncxx::builder::stream::finalize;
        auto result = poses.find_one(filter.view());
        if (result) {
            RCLCPP_ERROR(this->get_logger(), "Pose with Name: %s already exists", name.c_str());
            return false;
        }

        // Create a new Document
        auto doc = bsoncxx::builder::stream::document{};
        // Add the Name of the Pose to the Document
        doc << "name" << name;
        doc << "tcp_link" << moveGroupInterface->getEndEffectorLink();
        // Add the Position of the Pose to the Document
        doc << "position" << bsoncxx::builder::stream::open_array
            << pose.position.x
            << pose.position.y
            << pose.position.z
            << bsoncxx::builder::stream::close_array;
        // Add the Orientation of the Pose to the Document
        doc << "orientation" << bsoncxx::builder::stream::open_array
            << pose.orientation.x
            << pose.orientation.y
            << pose.orientation.z
            << pose.orientation.w
            << bsoncxx::builder::stream::close_array;

        // Insert the Document to the Collection
        poses.insert_one(doc.view());
        RCLCPP_INFO(this->get_logger(), "Inserted Pose with Name: %s", name.c_str());
        return true;
    }

    void TeleoperationBackendNode::moveRobotService(
            const std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Move Robot Service");
        printCurrentPose();
        // Get the Frame in which the Robot should move
        int frame = request->frame_id;
        if (frame == 0) {// Move Joint
            std::vector<double> joint_values(6, 0.0);
            // Set the vector to the old values
            joint_values = moveGroupInterface->getCurrentJointValues();
            // Find in the tokes first the axis and then the value
            for (int i = 0; i < request->data.size(); i++) {
                int axis = request->axis.at(i);
                double value = request->data.at(i);
                joint_values[axis] = value;
            }
            // Print out the Joint Values
            for (int i = 0; i < joint_values.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, joint_values[i]);
            }

            try {
                moveJoint(joint_values, false);
                response->success = true;
            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to move Robot");
                response->success = false;
                return;
            }

        } else if (frame == 1) {// Move World
            geometry_msgs::msg::Pose pose = moveGroupInterface->getCurrentPose().pose;
            // Add the values of the array to the current Pose
            // Print Info about Reqeust
            RCLCPP_INFO(this->get_logger(), "Size of data Vector is %d", request->data.size());
            RCLCPP_INFO(this->get_logger(), "Size of axis Vector is %d", request->axis.size());

            // Print the Current Pose
            RCLCPP_INFO(this->get_logger(), "Current Pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
                        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);

            for (int i = 0; i < request->data.size(); ++i) {
                int id = request->axis.at(i);
                switch (id) {
                    case 0:
                        pose.position.x += request->data.at(i);
                        break;
                    case 1:
                        pose.position.y += request->data.at(i);
                        break;
                    case 2:
                        pose.position.z += request->data.at(i);
                        break;
                    case 3:
                        changeOrientation(pose, request->data.at(i), 0);
                        break;
                    case 4:
                        changeOrientation(pose, request->data.at(i), 1);
                        break;
                    case 5:
                        changeOrientation(pose, request->data.at(i), 2);
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Invalid Axis ID");
                }
            }
            // Print the Current Pose
            RCLCPP_INFO(this->get_logger(), "New Pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
                        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
            try {
                moveWorldPose(pose);
                response->success = true;
            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to move Robot");
                response->success = false;
                return;
            }
        } else if (frame == 2) {
            // Move the Robot in the Tool Frame
            std::vector<float> distance = request->data;
            // Convert the Float Vector to a Double Vector
            std::vector<double> distance_double(distance.begin(), distance.end());
            std::vector<int> axis = request->axis;
            moveTCPPose(distance_double, axis);
            response->success = true;

        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid Frame ID");
            response->success = false;
            return;
        }
    }

    void TeleoperationBackendNode::selectToolService(
            const std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Select Tool Service");
        printCurrentPose();
        // Select the Tool
        switch (request->tcp_id) {
            case 0:
                // Flansch of the Robot Link_6_1
                moveGroupInterface->setEndEffectorLink("Link_6_1");
                response->success = true;
                response->message = "Selected Tool: Flansch";
                break;
            case 1:
                // Camera
                switchEndEffector(false);
                response->success = true;
                response->message = "Selected Tool: Camera";
                break;
            case 2:
                // Gripper
                switchEndEffector(true);
                response->success = true;
                response->message = "Selected Tool: Gripper";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid Tool ID");
                response->success = false;
                response->message = "Invalid Tool ID";
        }
    }

    void TeleoperationBackendNode::allignTCPService(
            const std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Allign TCP Service");
        printCurrentPose();
        // Align the TCP to the World Orientation
        // Check if pose is emtpy
        if (request->hs_pose == false) {
            RCLCPP_INFO(this->get_logger(), "Align %s to World Orientation",
                        moveGroupInterface->getEndEffectorLink().c_str());
            alignTCP();
            response->success = true;
            return;
        }
        response->success = false;
    }


    void TeleoperationBackendNode::teachPointService(
            const std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Teach Point Service");
        // Check if the Pose Name is home or
        std::regex pattern("home", std::regex_constants::icase);
        if (std::regex_match(request->name, pattern)) {
            std::cout << "The word is 'home' (case insensitive match)." << std::endl;
            response->success = false;
            response->message = "The word is 'home' (case insensitive match).";
            return;
        }
        auto pose = moveGroupInterface->getCurrentPose().pose;
        response->success = safePosetoMongoDB(pose, request->name);
    }

    void TeleoperationBackendNode::toolService(
            const std::shared_ptr<robot_teleoperation_interface::srv::Tool::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::Tool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Tool Service");
        // Get the Current Pose and display
        printCurrentPose();
        floatarray cmd;
        if (request->close) {
            // Close the Gripper by setting the first GPIO to One
            cmd.data.push_back(1.0);
        } else {
            // Open the Gripper by setting the first GPIO to Zero
            cmd.data.push_back(0.0);
        }
        gpio_publisher->publish(cmd);
        response->success = true;
        return;
    }

    void TeleoperationBackendNode::gpioCallback(const CmdType::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "GPIO Callback");
        // Limit the Callback execution to 1 Hz
        // Check if the GPIO is not empty
        if (msg->values.size() > 0) {
            // Check if the GPIO is the Homing GPIO
            if (msg->values[0] > 1.0) {
                gripper_open = true;
            } else {
                gripper_open = false;
            }
        }
    }

    void TeleoperationBackendNode::movePointService(
            const std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Move Point Service");
        RCLCPP_INFO(this->get_logger(), "Move Robot to Point: %s", request->point_name.c_str());

        // Check if point is named home
        if (request->point_name == "home") {
            homing = true;
            moveGroupInterface->setNamedTarget("home");
            auto result = moveGroupInterface->move();
            if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Move Successful");
                response->success = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Move Failed");
                response->success = false;
            }
            return;
        }

        // Get the Pose from the MongoDB
        auto filter = bsoncxx::builder::stream::document{} << "name" << request->point_name
                                                           << bsoncxx::builder::stream::finalize;
        auto result = poses.find_one(filter.view());
        if (result) {
            // Get the Pose from the Document
            auto view = result->view();

            // Get the Position of the Pose
            auto position = view["position"].get_array().value;
            geometry_msgs::msg::Point point;
            point.x = position[0].get_double();
            point.y = position[1].get_double();
            point.z = position[2].get_double();
            // Get the Orientation of the Pose
            auto orientation = view["orientation"].get_array().value;
            geometry_msgs::msg::Quaternion quaternion;
            quaternion.x = orientation[0].get_double();
            quaternion.y = orientation[1].get_double();
            quaternion.z = orientation[2].get_double();
            quaternion.w = orientation[3].get_double();
            // Create the Pose
            geometry_msgs::msg::Pose pose;
            pose.position = point;
            pose.orientation = quaternion;
            // Move the Robot to the Pose
            moveWorldPose(pose);
            response->success = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Pose not found");
            response->success = false;
        }
    }

    void TeleoperationBackendNode::printCurrentPose() {
        // Get the current Pose of the End Effector
        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current Pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);
    }

    void TeleoperationBackendNode::calculateSpeed_Timecallback() {

        const geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        rclcpp::Time current_time = this->now();
        if (last_time_.seconds() > 5.0) {
            double dt = 0.1;// 100ms
            double dx = current_pose.pose.position.x - last_position_.pose.position.x;
            double dy = current_pose.pose.position.y - last_position_.pose.position.y;
            double dz = current_pose.pose.position.z - last_position_.pose.position.z;

            // Convert to mm/s
            dx *= 1000;
            dy *= 1000;
            dz *= 1000;


            double linear_speed = sqrt(dx * dx + dy * dy + dz * dz) / dt;
            RCLCPP_DEBUG(get_logger(), "Linear Speed %f mm/s", linear_speed);
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = current_time;
            twist.twist.linear.x = dx / dt;
            twist.twist.linear.y = dy / dt;
            twist.twist.linear.z = dz / dt;
            speed_publisher_->publish(twist);
            pose_publisher_->publish(current_pose);
        }

        last_position_.pose = current_pose.pose;
        last_time_ = current_time;
    }
    void TeleoperationBackendNode::printPose(geometry_msgs::msg::Pose pose) {
        // Print the Current Pose with the Positon and Orientation in euler angles
        RCLCPP_INFO(this->get_logger(), "Current TCP-Pose: x: %f, y: %f, z: %f",
                    pose.position.x, pose.position.y, pose.position.z);
        // Transform the Orientation to Euler Angles
        tf2::Quaternion q;
        tf2::convert(pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // Convert the Euler Angles to Degree
        roll = roll * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        yaw = yaw * 180 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Moved TCP-Orientation: roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    }
    void TeleoperationBackendNode::moveTCPPose(std::vector<double> distance, std::vector<int> axis) {

        // Get the current Pose of the End Effector
        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        //TODO:  Transform the Pose to the Tool Frame
        geometry_msgs::msg::PoseStamped tool_pose;
        try {
            // Transform the Pose to the Tool Frame
            tool_pose = tf_buffer_->transform(current_pose, moveGroupInterface->getEndEffectorLink(), tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform to Tool Frame failed: %s", ex.what());
            return;
        }
        // Move the Pose in the Tool Frame
        for (int i = 0; i < distance.size(); ++i) {
            int axis_id = axis.at(i);
            switch (axis_id) {
                case 0:
                    tool_pose.pose.position.x += distance.at(i);
                    break;
                case 1:
                    tool_pose.pose.position.y += distance.at(i);
                    break;
                case 2:
                    tool_pose.pose.position.z += distance.at(i);
                    break;
                case 3:
                    changeOrientation(tool_pose.pose, distance.at(i), 0);
                    break;
                case 4:
                    changeOrientation(tool_pose.pose, distance.at(i), 1);
                    break;
                case 5:
                    changeOrientation(tool_pose.pose, distance.at(i), 2);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid Axis ID");
            }
        }

        //TODO: Transform the Pose back to the World Frame
        geometry_msgs::msg::PoseStamped new_world_pose;
        try {
            // Transform the Pose back to the World Frame
            new_world_pose = tf_buffer_->transform(tool_pose, "world", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform back to World Frame failed: %s", ex.what());
            return;
        }

        // Move the Robot to the new Pose
        moveWorldPose(new_world_pose.pose);
    }
}// namespace robo_teleoperation
// namespace robo_teleoperation

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<robo_teleoperation::TeleoperationBackendNode>());
    rclcpp::shutdown();
    return 0;
}