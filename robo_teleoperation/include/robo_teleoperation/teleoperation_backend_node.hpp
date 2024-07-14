/******************************************************************************
* Filename:    teleoperation_backend_node.hpp
* Description: Backend node for the teleoperation interface. This header file
*              defines the TeleoperationBackendNode class, which provides
*              functionalities for moving the robot to specific positions,
*              handling service requests, and interacting with MongoDB for
*              storing and retrieving robot poses. It also includes methods
*              for setting joint constraints, aligning the TCP, and handling
*              GPIO messages.
* Author:      Felix Pfeifer
* Email:       fpfeifer@stud.hs-heilbronn.de
* Version:     2.1
* License:     MIT License
******************************************************************************/


#ifndef ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
#define ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP

#include "robot_teleoperation_interface/msg/teleop.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <random>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Services
#include "robot_teleoperation_interface/srv/allign_tcp.hpp"
#include "robot_teleoperation_interface/srv/hand2_eye.hpp"
#include "robot_teleoperation_interface/srv/move_point.hpp"
#include "robot_teleoperation_interface/srv/move_robot.hpp"
#include "robot_teleoperation_interface/srv/select_tool.hpp"
#include "robot_teleoperation_interface/srv/teach_point.hpp"
#include "robot_teleoperation_interface/srv/tool.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "control_msgs/msg/interface_value.hpp"
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <regex>


// MongoDB
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/json.hpp>
#include <iostream>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/bulk_write_exception.hpp>
#include <mongocxx/exception/exception.hpp>
#include <mongocxx/exception/gridfs_exception.hpp>
#include <mongocxx/exception/logic_error.hpp>
#include <mongocxx/exception/query_exception.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/pipeline.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace robo_teleoperation {

    using CmdType = control_msgs::msg::InterfaceValue;
    using floatarray = std_msgs::msg::Float64MultiArray;

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
         * @param pose of the TCP in the World Frame
         */
        void moveWorldPose(geometry_msgs::msg::Pose pose);


        /**
        * Moves the robot's end effector (EE) frame along a specified axis by a given distance.
        *
        * @param distance The distance to move or rotate the end effector. For linear movements, this is in meters. For rotations,
        * this is in radians. The direction of movement or rotation is determined by the sign of the distance value: positive
        * values move or rotate the end effector in the positive direction of the specified axis, while negative values move or rotate
        * it in the opposite direction.
        * @param axis     The axis along which to move or rotate the end effector. This is specified as an integer value,
        * where 0 corresponds to the X-axis, 1 corresponds to the Y-axis, and 2 corresponds to the Z-axis for linear movements.
        * For rotations, 3 corresponds to roll (rotation around the X-axis), 4 corresponds to pitch (rotation around the Y-axis),
        * and 5 corresponds to yaw (rotation around the Z-axis) of the end effector frame.
        */
        void moveTCPPose(std::vector<double> distance, std::vector<int> axis);

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

        /**
         * Saves the current pose of the robot to the MongoDB
         * @param pose the current pose of the robot
         * @param name the name of the pose
         * @return true if the pose was saved successfully
         */
        bool safePosetoMongoDB(geometry_msgs::msg::Pose pose, std::string name);

        // Services

        /**
        * Moves the robot to the specified pose from the Service
        * @param request the request from the service
        * @param response the response from the service
        *
        */
        void moveRobotService(const std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Response> response);


        /**
         * Select the TCP of the Robot the Camera TCP or the Gripper TCP
         * @param request
         * @param response
         */
        void selectToolService(const std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Request> request,
                               std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Response> response);

        /**
         * Function to Align the TCP of the Robot with the World Frame
         * @param request
         * @param response
         */
        void allignTCPService(const std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Response> response);

        /**
         * Function to Teach a Point to the robot it saved to the mongodb
         * @param request
         * @param response
         */
        void teachPointService(const std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Request> request,
                               std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Response> response);

        /**
         * Function to open/close the gripper
         * @param request
         * @param response
         */
        void toolService(const std::shared_ptr<robot_teleoperation_interface::srv::Tool::Request> request,
                         std::shared_ptr<robot_teleoperation_interface::srv::Tool::Response> response);

        /**
         * Function to move the robot to a specified point from the MongoDB database
         * @param request
         * @param response
         */
        void movePointService(const std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Response> response);

        /**
         * Function to print the current pose of the robot
         */
        void printCurrentPose();

        /**
         * Print the Given Pose to the Terminal for Debugging
         * the Pose is printed in the format x, y, z, roll, pitch, yaw
         *
         * @param pose  the pose to be printed
         */
        void printPose(geometry_msgs::msg::Pose pose);

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

        /**
         * Changes the orientation of the given pose by the given angle around the given axis
         * @param pose the pose to be modified
         * @param angle the angle in radians
         * @param axis  the axis to rotate around (0 = x, 1 = y, 2 = z)
         */
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

        // Services from the Teleoperation Interface
        rclcpp::Service<robot_teleoperation_interface::srv::MoveRobot>::SharedPtr move_robot_service;
        rclcpp::Service<robot_teleoperation_interface::srv::MovePoint>::SharedPtr move_point_service;
        rclcpp::Service<robot_teleoperation_interface::srv::SelectTool>::SharedPtr select_tool_service;
        rclcpp::Service<robot_teleoperation_interface::srv::AllignTCP>::SharedPtr allign_tcp_service;
        rclcpp::Service<robot_teleoperation_interface::srv::Hand2Eye>::SharedPtr hand2_eye_service;
        rclcpp::Service<robot_teleoperation_interface::srv::TeachPoint>::SharedPtr teach_point_service;
        rclcpp::Service<robot_teleoperation_interface::srv::Tool>::SharedPtr tool_service;


        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::PlanningScene planning_scene;
        // Planning Scene
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;

        // Publisher for the GPIO Message of the Hardware Interface
        rclcpp::Publisher<floatarray>::SharedPtr gpio_publisher;
        rclcpp::Subscription<CmdType>::SharedPtr gpio_subscriber;

        /**
         * Callback function for the GPIO Subscriber
         * @param msg the GPIO message
         */
        void gpioCallback(const CmdType::SharedPtr msg);


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

        // MongoDB
        mongocxx::instance instance{};
        mongocxx::client client{mongocxx::uri{}};
        mongocxx::database db = client["Seminararbeit"];
        mongocxx::collection poses = db["Poses"];
        mongocxx::collection calib_poses = db["calibration_poses"];

        // Gripper State
        bool gripper_open;

        /**
         * Function to calculate speed of the end effector as a callback of Joint_States
         * @frequency : The frequency of 100Hz
         * @publish a TwistStamped message with the speed of the end effector
         */
        void calculateSpeed_Timecallback();

        // Last Pose of the eeframe
        geometry_msgs::msg::PoseStamped last_position_;
        // Publisher for the TwistStamped message
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_publisher_;
        // Publisher for the PoseStamped message
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

        // Last time initialized with 0
        rclcpp::Time last_time_ = rclcpp::Time(0, 0);

        // Subscriper of joint_states
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

        // Variables for the TCP Movement in the EndEffector Frame
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}// namespace robo_teleoperation
#endif//ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
