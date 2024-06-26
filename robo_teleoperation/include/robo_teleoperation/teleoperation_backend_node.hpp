/******************************************************************************
 * Filename:    teleoperation_backend_node.hpp
 * Description: Backend node for the teleoperation interface
 * Author:      Felix Pfeifer
 * Email:       fpfeifer@stud.hs-heilbronn.de
 * Version:     2.1
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

// Services
#include "robot_teleoperation_interface/srv/allign_tcp.hpp"
#include "robot_teleoperation_interface/srv/select_tool.hpp"
#include "robot_teleoperation_interface/srv/move_robot.hpp"
#include "robot_teleoperation_interface/srv/move_point.hpp"
#include "robot_teleoperation_interface/srv/hand2_eye.hpp"
#include "robot_teleoperation_interface/srv/teach_point.hpp"
#include "robot_teleoperation_interface/srv/tool.hpp"


#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>

// MongoDB
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

#include <chrono>
#include <memory>
#include <string>

namespace robo_teleoperation {

    using CmdType = std_msgs::msg::Float64MultiArray;

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

        /**
         * Saves the current pose of the robot to the MongoDB
         * @param pose the current pose of the robot
         * @param name the name of the pose
         */
        void safePosetoMongoDB(geometry_msgs::msg::Pose pose, std::string name);

        // Services

        /**
        * Moves the robot to the specified pose from the Service
        * @param request the request from the service
        * @param response the response from the service
        *
        */
        void moveRobotService(const std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::MoveRobot::Response> response);


        void selectToolService(const std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Request> request,
                               std::shared_ptr<robot_teleoperation_interface::srv::SelectTool::Response> response);

        void allignTCPService(const std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::AllignTCP::Response> response);

        void hand2EyeService(const std::shared_ptr<robot_teleoperation_interface::srv::Hand2Eye::Request> request,
                             std::shared_ptr<robot_teleoperation_interface::srv::Hand2Eye::Response> response);

        void teachPointService(const std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Request> request,
                               std::shared_ptr<robot_teleoperation_interface::srv::TeachPoint::Response> response);

        void toolService(const std::shared_ptr<robot_teleoperation_interface::srv::Tool::Request> request,
                         std::shared_ptr<robot_teleoperation_interface::srv::Tool::Response> response);

        void movePointService(const std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Request> request,
                              std::shared_ptr<robot_teleoperation_interface::srv::MovePoint::Response> response);

        void setupPlanningScene();

        // Subcriber the twist message
        void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;

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

        // Publisher for the GPIO Message of the Hardware Interface

        rclcpp::Publisher<CmdType>::SharedPtr gpio_publisher;
        rclcpp::Subscription<CmdType>::SharedPtr gpio_subscriber;

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

        // Planning Scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // MongoDB
        mongocxx::instance instance{};
        mongocxx::client client{mongocxx::uri{}};
        mongocxx::database db = client["Seminararbeit"];
        mongocxx::collection poses = db["Poses"];

        // Gripper State
        bool gripper_open;


    };
}
#endif //ROBO_TELEOPERATION_TELEOPERATION_BACKEND_NODE_HPP
