//
// Created by felix on 17.06.24.
//

#ifndef SENSOPART_VISION_ROS2_SENSOPART_VISION_NODE_HPP
#define SENSOPART_VISION_ROS2_SENSOPART_VISION_NODE_HPP

#include "sensopart_interfaces/msg/job.hpp"
#include "sensopart_interfaces/srv/calibration.hpp"
#include "sensopart_interfaces/srv/get_image.hpp"
#include "sensopart_interfaces/srv/get_jobs.hpp"
#include "sensopart_interfaces/srv/get_pose.hpp"
#include "sensopart_interfaces/srv/trigger_robotics.hpp"
#include "sensopart_interfaces/srv/set_job.hpp"

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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/node.hpp>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_interface.h>


#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <map>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/exception/exception.hpp>
#include <mongocxx/exception/logic_error.hpp>
#include <mongocxx/exception/query_exception.hpp>
#include <mongocxx/exception/bulk_write_exception.hpp>
#include <mongocxx/exception/gridfs_exception.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/pipeline.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <iostream>

class SensopartVisionNode : public rclcpp::Node
{
private:
    // Service Clients for the Sensopart Vision System
    rclcpp::Client<sensopart_interfaces::srv::Calibration>::SharedPtr calibration_service_;
    rclcpp::Client<sensopart_interfaces::srv::GetImage>::SharedPtr get_image_service_;
    rclcpp::Client<sensopart_interfaces::srv::GetJobs>::SharedPtr get_jobs_service_;
    rclcpp::Client<sensopart_interfaces::srv::SetJob>::SharedPtr set_job_service_;
    rclcpp::Client<sensopart_interfaces::srv::TriggerRobotics>::SharedPtr trigger_robotics_service_;


    // Service Servers for the Sensopart Vision System to get a Pose from the teleop Packet
    rclcpp::Service<sensopart_interfaces::srv::GetPose>::SharedPtr get_pose_service_;

    // MoveIt
    moveit::planning_interface::MoveGroupInterfacePtr moveGroupInterface;
    rclcpp::executors::SingleThreadedExecutor executor_;

    // Joint Constraints
    moveit_msgs::msg::JointConstraint joint_constraint_;

    // List of Calibration Poses
    std::vector<geometry_msgs::msg::Pose> calibration_poses_;

    // MongoDB
    mongocxx::instance instance{};
    mongocxx::client client{mongocxx::uri{}};
    mongocxx::database db = client["Seminararbeit"];
    mongocxx::collection collection = db["calibration_poses"];


    void checkCalibrationPoses();
    // MongoDB get all Poses for the Hand to Eye Calibration
    std::vector<geometry_msgs::msg::Pose> getCalibrationPosesFromDB();

    std::vector<double> poseToVector(geometry_msgs::msg::Pose pose);

    // Moveit Node for the Sensopart Vision System
    rclcpp::Node::SharedPtr node;

public:

    SensopartVisionNode();

    void setupJointConstraints();

    void handToEyeCalibration();

};

#endif //SENSOPART_VISION_ROS2_SENSOPART_VISION_NODE_HPP
