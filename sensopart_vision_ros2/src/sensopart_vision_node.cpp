
#include "../include/sensopart_vision_ros2/sensopart_vision_node.hpp"

SensopartVisionNode::SensopartVisionNode() : rclcpp::Node("sensopart_vision_node") {

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("sensopart_moveit", node_options);

    executor_.add_node(node);
    std::thread([&]() { executor_.spin(); }).detach();

    moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robot");

    // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
    moveGroupInterface->setPlannerId("RRTConnect");
    setupJointConstraints();


    // Setup Server Client for Calibration
    calibration_service_ = this->create_client<sensopart_interfaces::srv::Calibration>(
            "/sensopart_connector/calibrate");
    get_jobs_service_ = this->create_client<sensopart_interfaces::srv::GetJobs>("/sensopart_connector/get_job_list");
    trigger_robotics_service_ = this->create_client<sensopart_interfaces::srv::TriggerRobotics>(
            "/sensopart_connector/trigger_with_position");
    set_job_service_ = this->create_client<sensopart_interfaces::srv::SetJob>("/sensopart_connector/change_job_permanent");

    // Wait for all Services to be available
    while (!calibration_service_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service %s not available, waiting again...",
                    calibration_service_->get_service_name());
    }
    while (!get_jobs_service_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service %s not available, waiting again...",
                    get_jobs_service_->get_service_name());
    }
    while (!trigger_robotics_service_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service %s not available, waiting again...",
                    trigger_robotics_service_->get_service_name());
    }
    RCLCPP_INFO(this->get_logger(), "All services are available");

    // Move to the Home Position
    moveGroupInterface->setNamedTarget("home");
    moveGroupInterface->move();

    // Select the Tool TCP
    moveGroupInterface->setEndEffectorLink("tcp_gripper");
    // Select the camera_tcp as the end effector
    //checkCalibrationPoses();
}

// Method to check if the robot can move all calibration Poses
void SensopartVisionNode::checkCalibrationPoses() {
    // Get all Poses from the MongoDB
    calibration_poses_ = getCalibrationPosesFromDB();

    // Check if the calibration Poses

    // Move to the first Pose
    // Move to the next Pose and
    for (int i = 0; i < calibration_poses_.size(); ++i) {
        // Print the Pose
        RCLCPP_INFO(this->get_logger(), "Pose x: %f y: %f z: %f", calibration_poses_[i].position.x,
                    calibration_poses_[i].position.y, calibration_poses_[i].position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation roll: %f pitch: %f yaw: %f", calibration_poses_[i].orientation.x,
                    calibration_poses_[i].orientation.y, calibration_poses_[i].orientation.z);
        RCLCPP_INFO(this->get_logger(), "Moving to Pose %d", i);
        moveGroupInterface->setStartState(*moveGroupInterface->getCurrentState());
        moveGroupInterface->setPoseTarget(calibration_poses_[i]);
        moveGroupInterface->move();
    }
}

void SensopartVisionNode::handToEyeCalibration() {
    RCLCPP_INFO(this->get_logger(), "Starting the Hand to Eye Calibration");
    // Change the job to job 1 permanent
    auto request_job = std::make_shared<sensopart_interfaces::srv::SetJob::Request>();
    request_job->job = 1;
    auto result_job = set_job_service_->async_send_request(request_job);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_job);

    // Get all Poses from the MongoDB
    calibration_poses_ = getCalibrationPosesFromDB();

    // Init the Calibration with request
    auto request = std::make_shared<sensopart_interfaces::srv::Calibration::Request>();
    request->init = true;
    request->first = false;
    request->compute = false;
    auto result = calibration_service_->async_send_request(request);
    // Spin until the result is available
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Calibration initialized");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration not initialized");
        return;
    }

    // Move to the first Pose
    RCLCPP_INFO(this->get_logger(), "Moving to the first Pose");
    // Print the Pose to the first Pose
    RCLCPP_INFO(this->get_logger(), "Pose x: %f y: %f z: %f", calibration_poses_[0].position.x,
                calibration_poses_[0].position.y, calibration_poses_[0].position.z);


    moveGroupInterface->setPoseTarget(calibration_poses_[0]);
    moveGroupInterface->move();
    auto current_pose = moveGroupInterface->getCurrentPose();
    // Print current End Effector Pose with link name
    RCLCPP_INFO(this->get_logger(), "Current End Effector %s", moveGroupInterface->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Current Pose x: %f y: %f z: %f", current_pose.pose.position.x,
                current_pose.pose.position.y, current_pose.pose.position.z);
    // Get the current Pose

    // Format the Pose to a list with axis and angle in mm and degrees xyz and rpy
    std::vector<double> poses = poseToVector(current_pose.pose);
    // Print Values from the Pose Vector
    RCLCPP_INFO(this->get_logger(), "Pose x: %f y: %f z: %f", poses[0], poses[1], poses[2]);
    // Call the Calibration Service
    request = std::make_shared<sensopart_interfaces::srv::Calibration::Request>();
    request->init = false;
    request->first = true;
    request->compute = false;
    request->pose = poses;
    result = calibration_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Calibration first Pose");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration error with the first Pose");
        return;
    }
    // Move to the next Pose and
    for (int i = 1; i < calibration_poses_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Moving to Pose %d", i);
        // Print the Pose
        RCLCPP_INFO(this->get_logger(), "Pose x: %f y: %f z: %f", calibration_poses_[i].position.x,
                    calibration_poses_[i].position.y, calibration_poses_[i].position.z);
        moveGroupInterface->setPoseTarget(calibration_poses_[i]);
        moveGroupInterface->move();
        // Get the current Pose
        current_pose = moveGroupInterface->getCurrentPose();
        // Format the Pose to a list with axis and angle in mm and degrees xyz and rpy
        std::vector<double> poses = poseToVector(current_pose.pose);
        // Call the Calibration Service
        request = std::make_shared<sensopart_interfaces::srv::Calibration::Request>();
        request->init = false;
        request->first = false;
        request->compute = false;
        request->pose = poses;
        result = calibration_service_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
        if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Calibration Pose %d", i);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Calibration error with Pose %d", i);
            return;
        }

    }
    // Compute the Calibration
    request = std::make_shared<sensopart_interfaces::srv::Calibration::Request>();
    request->init = false;
    request->first = false;
    request->compute = true;
    result = calibration_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Calibration computed");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration not computed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Hand to Eye Calibration finished");

}

void SensopartVisionNode::setupJointConstraints() {

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

std::vector<geometry_msgs::msg::Pose> SensopartVisionNode::getCalibrationPosesFromDB() {
    // Connect to the DB and get all Entrys currently in the DB are 7 Poses
    std::vector<geometry_msgs::msg::Pose> poses;

    auto cursor = collection.find({});
    // Iterate over the Cursor and get all Poses
    // format in the DB name, x, y, z, roll,pitch,yaw in degrees and mm
    for (auto &&doc: cursor) {
        // Name is the order of the Poses
        // Position and Orientation of the Pose are a list with axis and angle in mm and degrees xyz and rpy
        std::string name = doc["name"].get_utf8().value.to_string();
        auto axis = doc["position"].get_array().value;
        // Print the axis View
        double x = axis[0].get_double().value;
        double y = axis[1].get_double().value;
        double z = axis[2].get_double().value;
        RCLCPP_INFO(this->get_logger(), "Axis x: %f y: %f z: %f", x, y, z);
        // Convert the xyz in mm to meter
        x = x / 1000;
        y = y / 1000;
        z = z / 1000;

        // Angels are in degrees in the doc Orientation
        auto orientation = doc["orientation"].get_array().value;
        double roll = orientation[0].get_double().value;
        double pitch = orientation[1].get_double().value;
        double yaw = orientation[2].get_double().value;
        RCLCPP_INFO(this->get_logger(), "Orientation roll: %f pitch: %f yaw: %f", roll, pitch, yaw);


        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        tf2::Quaternion q;
        // Convert the angels to radians
        roll = roll * M_PI / 180;
        pitch = pitch * M_PI / 180;
        yaw = yaw * M_PI / 180;

        q.setRPY(roll, pitch, yaw);
        pose.orientation = tf2::toMsg(q);
        calibration_poses_.push_back(pose);
    }
    return calibration_poses_;
}

std::vector<double> SensopartVisionNode::poseToVector(geometry_msgs::msg::Pose pose) {
    std::vector<double> poses;

    // Convert the given pose to a list with axis in mm and angles in degrees
    // Spec in Doc axis unit * 1000 and angles in degrees * 1000

    // Multiply position values by 1000 to convert from meters to millimeters
    poses.push_back(pose.position.x * 1000);
    poses.push_back(pose.position.y * 1000);
    poses.push_back(pose.position.z * 1000);

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Convert the angles to degrees and multiply by 1000
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    poses.push_back(roll);
    poses.push_back(pitch);
    poses.push_back(yaw);

    return poses;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensopartVisionNode>();

    node->handToEyeCalibration();
    rclcpp::shutdown();
    return 0;
}

