
#include "../include/sensopart_vision_ros2/sensopart_vision_node.hpp"

SensopartVisionNode::SensopartVisionNode() : rclcpp::Node("sensopart_vision_node") {

    auto node = std::make_shared<rclcpp::Node>("senso_moveit_node");

    moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robot");

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("move_group_node", node_options);

    executor_.add_node(node);
    std::thread([&]() { executor_.spin(); }).detach();

    moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robot");

    // Use to move the Robot the OMPL Planner with the RRTConnect Algorithm
    moveGroupInterface->setPlannerId("RRTConnect");

    setupJointConstraints();


    // Setup Server Client for Calibration
    calibration_service_ = this->create_client<sensopart_interfaces::srv::Calibration>("calibration");
    get_image_service_ = this->create_client<sensopart_interfaces::srv::GetImage>("get_image");
    get_jobs_service_ = this->create_client<sensopart_interfaces::srv::GetJobs>("get_jobs");
    set_job_service_ = this->create_client<sensopart_interfaces::srv::SetJob>("set_job");


}

void SensopartVisionNode::handToEyeCalibration() {

    // Load the Poses from the MongoDB
    // Select all Poses contains in the name "hand2eye" from the Collection "Poses"
    auto cursor = collection.find(
            bsoncxx::builder::stream::document{} << "name"
                                                 << bsoncxx::builder::stream::open_document << "$regex"
                                                 << "hand2eye" << bsoncxx::builder::stream::close_document
                                                 << bsoncxx::builder::stream::finalize);

    for (auto &&doc: cursor) {

        // Get the Pose from the cursor
        auto pose = doc["pose"];

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose["x"].get_double().value;
        pose_msg.position.y = pose["y"].get_double().value;
        pose_msg.position.z = pose["z"].get_double().value;
        pose_msg.orientation.x = pose["qx"].get_double().value;
        pose_msg.orientation.y = pose["qy"].get_double().value;
        pose_msg.orientation.z = pose["qz"].get_double().value;
        pose_msg.orientation.w = pose["qw"].get_double().value;
        calibration_poses_.push_back(pose_msg);
    }

    // If there are not enough Poses in the MongoDB, the Calibration will be aborted
    if (calibration_poses_.size() < 10) {
        RCLCPP_FATAL(this->get_logger(), "Not enough Poses in the MongoDB");
        return;
    }



    // Init the calibration service
    auto request = std::make_shared<sensopart_interfaces::srv::Calibration::Request>();
    request->init = true;
    auto result = calibration_service_->async_send_request(request);


    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Calibration Service: %s",
                    result.get()->success ? "Success" : "Failed");
    }


    // Moves to the 10 Poses for the Calibration
    for (int i = 0; i < 10; ++i) {

        // Get the Pose from the List of Poses
        auto target_pose = calibration_poses_[i];
        moveGroupInterface->setPoseTarget(target_pose);
        moveGroupInterface->move();


        auto pose = moveGroupInterface->getCurrentPose();
        request->first = false;

        if (i == 0) {
            request->first = true;
        }

        // Send the First Pose of the Calibration
        request->init = false;

        // Create a Vector of Axis and Angles in mm and degrees of the tcp

        std::vector<double> axis = {pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z};
        // Convert the Quaternion to Euler Angles
        tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                          pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        std::vector<double> angles = {roll, pitch, yaw};

        axis.push_back(angles[0]);
        axis.push_back(angles[1]);
        axis.push_back(angles[2]);

        request->pose = axis;
        result = calibration_service_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Calibration Service: %s",
                        result.get()->success ? "Success" : "Failed");
        }

    }

}

void SensopartVisionNode::setupJointConstraints() {

    // Limit the Joints 1 and 4 to +-90 deg roation
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


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensopartVisionNode>();
    node->handToEyeCalibration();

    rclcpp::shutdown();
    return 0;
}

