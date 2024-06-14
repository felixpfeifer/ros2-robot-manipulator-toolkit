/******************************************************************************
 * Filename:    teleoperation_backend_node.hpp
 * Description: Backend node for the teleoperation interface
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     2.0
 * License:     MIT License
 ******************************************************************************/

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
        void moveNamedPositon(const std::string &position);

        /**
         * Moves the robot to the specified pose
         * @param msg the teleoperation message
         *
         */
        void moveRobot(robot_teleoperation_interface::msg::Teleop::SharedPtr msg);

        /**
         * Moves the robot to the specified joint values
         * @param joint_values
         */
        void moveJoint(std::vector<double> joint_values, bool jog);

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
         * Limits the Joint 1 and 4 to move in a specific range -90° to 90° due
         * the behavior of the robot to flip the axis 1 and 4 when the robot is moved
         *
         */
        void setJointConstraints();

    private:
        /**
         * Callback function for the timer
         * moves the robot to a random position or the home position
         *
         */
        void timer_callback();

        /**
         * Switches the end effector between the gripper and the camera
         * @param gripper if true the gripper is selected, otherwise the camera
         */
        void switchEndEffector(bool gripper);

        /**
         * Aligns the TCP with the World Frame axis
         * only the orientation of the TCP is changed
         */
        void alignTCP(void);

        void changeOrientation(geometry_msgs::msg::Pose &pose, double angle, int axis);

        /**
         * Changes the orientation of the target pose adding the given roll, pitch and yaw values
         * to the current orientation
         * @param target_pose the target pose to be modified
         * @param roll angle in radians (rotation around x-axis)
         * @param pitch angle in radians (rotation around y-axis)
         * @param yaw angle in radians  (rotation around z-axis)
         * @return the modified target_pose with the new orientation
         */
        geometry_msgs::msg::Pose &
        changeOrientation(geometry_msgs::msg::Pose &target_pose, double roll, double pitch, double yaw);

        // Publischer for the Teleoperation Command and Subscriper
        rclcpp::Publisher<robot_teleoperation_interface::msg::Teleop>::SharedPtr teleoperation_interface_publisher;
        rclcpp::Subscription<robot_teleoperation_interface::msg::Teleop>::SharedPtr teleoperation_interface_subscription;

        // Constants for the MoveGroupInterface
        const std::string PLANNING_GROUP = "robot";
        const std::string GRIPPER_LINK = "tcp_gripper";
        const std::string CAMERA_LINK = "camera_tcp";

        rclcpp::Logger logger = get_logger();
        rclcpp::Node::SharedPtr node;
        moveit::planning_interface::MoveGroupInterfacePtr moveGroupInterface;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool homing;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    };
}
#endif //ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
