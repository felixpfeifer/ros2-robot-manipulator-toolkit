//
// Created by felix on 06.06.24.
//

#ifndef ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
#define ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP

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
#include "robot_teleoperation_interface/msg/teleop.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/node.hpp>
#include <random>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_interface.h>


#include <chrono>
#include <memory>
#include <string>

namespace robo_teleoperation {

    class TeleoperationBackendNode : public rclcpp::Node {
    public:
        TeleoperationBackendNode();

        ~TeleoperationBackendNode();

        /**
         * Moves the robot to the home position
         */
        void moveNamedPositon(const std::string& position);

        void moveRobot(robot_teleoperation_interface::msg::Teleop::SharedPtr msg);

        /**
         * Moves the robot to the specified joint values
         * @param joint_values
         */
        void moveJoint(std::vector<double> joint_values);

        /**
         * Moves the robot to the specified pose in the World Frame
         * @param pose
         */
        void moveWorldPose(geometry_msgs::msg::Pose pose);

        /**
         * Moves the robot to the specified pose in the TCP Frame
         * @param pose
         */
        void moveTCPPose(geometry_msgs::msg::Pose pose);

        /**
         * Moves the robot to a random position in the World Frame
         */
        void moveRandom();

        /**
         * Sets the joint constraints for the robot
         *
         */
        void setJointConstraints();

    private:
        void timer_callback();
        rclcpp::Logger logger = get_logger();
        rclcpp::Node::SharedPtr node;
        moveit::planning_interface::MoveGroupInterfacePtr moveGroupInterface;
        moveit::core::JointModelGroup *jointModelGroup;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::TimerBase::SharedPtr timer_;
        const std::string PLANNING_GROUP = "robot";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        bool homing;

        // Publischer for the Teleoperation Command and Subscriper
        rclcpp::Publisher<robot_teleoperation_interface::msg::Teleop>::SharedPtr teleoperation_interface_publisher;
        rclcpp::Subscription<robot_teleoperation_interface::msg::Teleop>::SharedPtr teleoperation_interface_subscription;
        geometry_msgs::msg::Pose getPose(std::vector<double> value);
        geometry_msgs::msg::Pose getPose(std::vector<double> value, geometry_msgs::msg::Pose pose);
    };
}
#endif //ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
