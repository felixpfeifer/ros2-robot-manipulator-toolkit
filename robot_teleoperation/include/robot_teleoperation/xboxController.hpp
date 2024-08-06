//
// Created by felix on 19.06.24.
//

#ifndef robot_teleoperation_XBOXCONTROLLER_HPP
#define robot_teleoperation_XBOXCONTROLLER_HPP

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "robot_teleoperation_interface/srv/select_tool.hpp"
#include "robot_teleoperation_interface/srv/tool.hpp"

// Import rcl_interfaces for parameter setting
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/parameter_client.hpp>


using Parameter = rcl_interfaces::msg::Parameter;

class xbox_controller : public rclcpp::Node {
private:
    // We'll just set up parameters here
    const std::string JOY_TOPIC = "/joy";
    const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
    const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
    const std::string EEF_FRAME_ID = "TCP_Gripper";
    const std::string BASE_FRAME_ID = "world";


    // Enums for button names -> axis/button array index
    // For XBOX 1 controller
    enum Axis {
        LEFT_STICK_X = 0,
        LEFT_STICK_Y = 1,
        LEFT_TRIGGER = 2,
        RIGHT_STICK_X = 3,
        RIGHT_STICK_Y = 4,
        RIGHT_TRIGGER = 5,
        D_PAD_X = 6,
        D_PAD_Y = 7
    };
    enum Button {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        LEFT_BUMPER = 4,
        RIGHT_BUMPER = 5,
        CHANGE_VIEW = 6,
        MENU = 7,
        HOME = 8,
        LEFT_STICK_CLICK = 9,
        RIGHT_STICK_CLICK = 10
    };

    // Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
    // This will map the default values for the axes
    std::map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER, 1.0},
                                            {RIGHT_TRIGGER, 1.0}};
    std::map<Button, double> BUTTON_DEFAULTS;

    bool is_gripper_open = false;
    // Vector of int used for debouncing the 8 states
    std::map<std::string, int> debouncing_states = {
            {"A", 0},
            {"B", 0},
            {"X", 0},
            {"Y", 0},
            {"UP", 0},
            {"DOWN", 0},
            {"LEFT", 0},
            {"RIGHT", 0}};
    // Service Client for the Teleop Backend
    // Select Tool TCP for the Teleop Backend
    rclcpp::Client<robot_teleoperation_interface::srv::SelectTool>::SharedPtr select_tool_client;
    // Open Close the Gripper Tool
    rclcpp::Client<robot_teleoperation_interface::srv::Tool>::SharedPtr tool_client;

    // Future for the service call
    rclcpp::Client<robot_teleoperation_interface::srv::Tool>::SharedFuture tool_future;
    rclcpp::Client<robot_teleoperation_interface::srv::SelectTool>::SharedFuture select_tool_future;

    // Callback for the service call
    void tool_callback(rclcpp::Client<robot_teleoperation_interface::srv::Tool>::SharedFuture future);
    void select_tool_callback(rclcpp::Client<robot_teleoperation_interface::srv::SelectTool>::SharedFuture future);

    // Publisher for the twist command
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    // Publisher for the joint command
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    // Subscriber for the joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Services to interact with Servo
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    // Service to stop the ServoNode
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;

    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;


    bool convertJoyToCmd(const std::vector<float> &axes, const std::vector<int> &buttons,
                         std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist);

    const double MAX_ROT_SPEED = 0.5;
    double max_rot = 0.5;
    double currentAngularPercentage = 1.0;
    const double MAX_VEL = 0.25;
    double max_vel = 0.25;
    double currentLinearVelocityPercent = 1.0;

    rclcpp::Node::SharedPtr node;

public:
    xbox_controller();

    ~xbox_controller();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};


#endif//robot_teleoperation_XBOXCONTROLLER_HPP
