/******************************************************************************
 * Filename:    teleoperation_backend_node.cpp
 * Description: Backend node for the teleoperation interface
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     2.1
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
        gpio_publisher = this->create_publisher<CmdType>("/gpio_controller/commands", 10);
        gpio_subscriber = this->create_subscription<CmdType>(
                "/gpio_controller/inputs", 10,
                std::bind(&TeleoperationBackendNode::gpioCallback, this, std::placeholders::_1));

        // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
        moveGroupInterface->setPlannerId("RRTConnect");


        // Create the planning_scene_publisher
        planning_scene_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>(
                "/planning_scene", 10);

        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is starting");

        // Set Joint Constraints to avoid flipping
        setJointConstraints();
        // Create a Planning Scene
        setupPlanningScene();

        gripper_open = true;

        // Print Current Planning Frame
        RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", moveGroupInterface->getPlanningFrame().c_str());
        setJointConstraints();

        // Teleop Controller is ready to receive commands
        RCLCPP_INFO(this->get_logger(), "Teleoperation Backend Node is ready to receive commands");

        // Timer that runs every 20ms to get the current state of the robot
        //timer_ = this->create_wall_timer(1s, std::bind(&TeleoperationBackendNode::timer_callback, this));

        // Move to Home Position
        moveNamedPositon("home");
        homing = false;
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
        moveGroupInterface->move();
    }

    void TeleoperationBackendNode::moveWorldPose(geometry_msgs::msg::Pose pose) {
        RCLCPP_INFO(this->get_logger(), "Move Robot to World Pose");
        moveGroupInterface->setStartState(*moveGroupInterface->getCurrentState());
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
            moveJoint(joint_values, false);

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
            moveWorldPose(pose);
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
        auto pose = moveGroupInterface->getCurrentPose().pose;
        response->success = safePosetoMongoDB(pose, request->name);
    }

    void TeleoperationBackendNode::toolService(
            const std::shared_ptr<robot_teleoperation_interface::srv::Tool::Request> request,
            std::shared_ptr<robot_teleoperation_interface::srv::Tool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Tool Service");
        // Get the Current Pose and display
        printCurrentPose();
        CmdType cmd;
        if (request->close) {
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
        RCLCPP_INFO(this->get_logger(), "GPIO Callback");
        // Check if the GPIO is not empty
        if (msg->data.size() > 0) {
            // Check if the GPIO is the Homing GPIO
            if (msg->data[0] > 1.0) {
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

    void TeleoperationBackendNode::setupPlanningScene() {
        // Add a Collision Plane under the Robot to avoid collision with the ground
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = moveGroupInterface->getPlanningFrame();
        collision_object.id = "ground_plane";

        // Define the plane dimensions (in meters)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 10.0;// length
        primitive.dimensions[1] = 10.0;// width
        primitive.dimensions[2] = 0.01;// height

        // Define the pose of the plane (relative to the planning frame)
        geometry_msgs::msg::Pose plane_pose;
        plane_pose.orientation.w = 1.0;
        plane_pose.position.z = -0.005;// half of the height

        // Add the primitive and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(plane_pose);
        collision_object.operation = collision_object.ADD;

        // Create a vector of collision objects
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // Publish the collision objects
        planning_scene_interface.addCollisionObjects(collision_objects);

        RCLCPP_INFO(this->get_logger(), "Added Ground Plane to Planning Scene");
        addPLT();
        RCLCPP_INFO(this->get_logger(), "Added PLT to Planning Scene");
        addBox();
        RCLCPP_INFO(this->get_logger(), "Added Box to Planning Scene");
        addCalibrationPlate();
        RCLCPP_INFO(this->get_logger(), "Added Calibration Plate to Planning Scene");
    }

    bool TeleoperationBackendNode::addPLT() {
        // object_id = 1
        // Add the Calibration Plate to the Planning Scene
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = moveGroupInterface->getPlanningFrame();
        collision_object.id = "plt";
        shapes::Shape *plt = shapes::createMeshFromResource(
                "package://robo_teleoperation/meshes/palette.stl");
        RCLCPP_INFO(this->get_logger(), "Loaded Mesh");
        shape_msgs::msg::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(plt, mesh_msg);
        mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);
        geometry_msgs::msg::Pose plt_pose;
        shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
        plt_pose = getObjectPose(1, meshet);
        RCLCPP_INFO(this->get_logger(), "Got Pose");
        collision_object.mesh_poses.push_back(plt_pose);
        collision_object.operation = collision_object.ADD;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objectsPLT;
        collision_objectsPLT.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objectsPLT);
        return true;
    }

    bool TeleoperationBackendNode::addBox() {
        // object_id = 2
        // Add the Box to the Planning Scene
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = moveGroupInterface->getPlanningFrame();
        collision_object.id = "kiste";
        shapes::Shape *plt = shapes::createMeshFromResource(
                "package://robo_teleoperation/meshes/kiste.stl");

        shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
        shape_msgs::msg::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(plt, mesh_msg);
        mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);
        geometry_msgs::msg::Pose plt_pose;
        plt_pose = getObjectPose(3, meshet);
        collision_object.mesh_poses.push_back(plt_pose);
        collision_object.operation = collision_object.ADD;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objectsPLT;
        collision_objectsPLT.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objectsPLT);
        return true;
    }

    bool TeleoperationBackendNode::addCalibrationPlate() {
        // object_id = 2
        // Add the Plate to the Planning Scene
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = moveGroupInterface->getPlanningFrame();
        collision_object.id = "platte";
        shapes::Shape *plt = shapes::createMeshFromResource(
                "package://robo_teleoperation/meshes/kalibierplatte.stl");
        shape_msgs::msg::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(plt, mesh_msg);
        mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);
        geometry_msgs::msg::Pose plt_pose;
        shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
        plt_pose = getObjectPose(2, meshet);
        collision_object.mesh_poses.push_back(plt_pose);
        collision_object.operation = collision_object.ADD;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objectsPLT;
        collision_objectsPLT.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objectsPLT);
        return true;
    }


    std::vector<geometry_msgs::msg::Point> TeleoperationBackendNode::getColisionCorners(int id) {
        std::vector<geometry_msgs::msg::Point> point_list;
        std::vector<int> order;
        // resize the vector to 2
        // Load the Collision Object from the MongoDB DB with collection collision_objects
        // Withh the colision "object_id"
        auto filter = bsoncxx::builder::stream::document{} << "object_id" << id
                                                           << bsoncxx::builder::stream::finalize;
        RCLCPP_INFO(this->get_logger(), "Get Collision Object with ID: %d", id);
        // Find all Points of the Collision Object
        auto result = collision_objects.find(filter.view());
        RCLCPP_INFO(this->get_logger(), "Found Collision Object with ID: %d", id);
        // Check if the cursor is not empty and process the results
        for (auto &&doc: result) {
            // Parse the BSON document to extract the points
            // In the Document is a var point_number with the number of points
            // Get the number of the point
            int point_number = doc["point_number"].get_int32();

            if (doc["points"]) {
                auto points = doc["points"].get_array().value;
                double x = points[0].get_double();
                double y = points[1].get_double();
                double z = points[2].get_double();
                // Convert to a Point Message
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                point_list.push_back(p);
                order.push_back(point_number);
                // Do something with the point (e.g., log or use in collision object)
                RCLCPP_INFO(this->get_logger(), "Point position: [x: %f, y: %f, z: %f]", x, y, z);
            }
        }
        // Sort the Points after the id in the order vector
        std::vector<geometry_msgs::msg::Point> sorted_points;
        if (order[0] < order[1]) {
            sorted_points.push_back(point_list[0]);
            sorted_points.push_back(point_list[1]);
        } else {
            sorted_points.push_back(point_list[1]);
            sorted_points.push_back(point_list[0]);
        }
        return sorted_points;
    }

    geometry_msgs::msg::Pose
    TeleoperationBackendNode::getObjectPose(int id, shapes::Mesh *mesh) {

        // Pose of the Object
        geometry_msgs::msg::Pose box_center_pose;

        // Size of the Object
        std::vector<double> size = getSTLSize(mesh);
        double length = size[0];
        double width = size[1];
        double height = size[2];


        // Get the two connected corners of the object
        std::vector<geometry_msgs::msg::Point> corners = getColisionCorners(id);

        // Ensure we have exactly two points
        if (corners.size() != 2) {
            throw std::runtime_error("Expected exactly two corner points");
        }

        double dx = corners[1].x - corners[0].x;
        double dy = corners[1].y - corners[0].y;
        double distance_between_corners = sqrt(dx * dx + dy * dy);

        double edge_length = 0;
        double other_edge_length = 0;
        // Determine if the given edge is the width or the length of the object
        if (abs(distance_between_corners - width) < abs(distance_between_corners - length)) {
            edge_length = width;
            other_edge_length = length;
        } else {
            edge_length = length;
            other_edge_length = width;
        }
        // Calulate the orientation of the object with angel theta
        double theta = atan2(dy, dx);
        // Calculate the normal vector of the given edge with the eigen lib
        Eigen::Vector2d normal_vector(-dy, dx);
        normal_vector = normal_vector / distance_between_corners;
        // Calculate the center of the object
        Eigen::Vector2d center_offset = (other_edge_length / 2) * normal_vector;
        auto x_center = (corners[0].x + corners[1].x) / 2 + center_offset[0];
        auto y_center = (corners[0].y + corners[1].y) / 2 + center_offset[1];

        // Calculate the half-dimensions
        double half_edge = edge_length / 2;
        double half_other_edge = other_edge_length / 2;

        // Set the position of the object
        box_center_pose.position.x = x_center;
        box_center_pose.position.y = y_center;
        box_center_pose.position.z = 0;// Assume the object is on the ground

        // Set the orientation of the object
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        tf2::convert(q, box_center_pose.orientation);

        return box_center_pose;
    }

    std::vector<double> TeleoperationBackendNode::getSTLSize(shapes::Mesh *mesh) {
        std::vector<double> size;

        // Extract the dimensions of the STL model
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::min();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::min();
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::min();

        for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
            const double *vertex = &mesh->vertices[3 * i];
            if (vertex[0] < min_x) min_x = vertex[0];
            if (vertex[0] > max_x) max_x = vertex[0];
            if (vertex[1] < min_y) min_y = vertex[1];
            if (vertex[1] > max_y) max_y = vertex[1];
            if (vertex[2] < min_z) min_z = vertex[2];
            if (vertex[2] > max_z) max_z = vertex[2];
        }

        double length = max_x - min_x;
        double width = max_y - min_y;
        double height = max_z - min_z;

        size.push_back(length);
        size.push_back(width);
        size.push_back(height);

        return size;
    }

    void TeleoperationBackendNode::printCurrentPose() {
        // Get the current Pose of the End Effector
        geometry_msgs::msg::PoseStamped current_pose = moveGroupInterface->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current Pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);
    }

}// namespace robo_teleoperation

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<robo_teleoperation::TeleoperationBackendNode>());
    rclcpp::shutdown();
    return 0;
}