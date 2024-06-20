//
// Created by felix on 19.06.24.
//

#ifndef ROBO_TELEOPERATION_XBOXCONTROLLER_HPP
#define ROBO_TELEOPERATION_XBOXCONTROLLER_HPP

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
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
#include <thread>


class xbox_controller : public rclcpp::Node {
private:

    // We'll just set up parameters here
    const std::string JOY_TOPIC = "/joy";
    const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
    const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
    const std::string EEF_FRAME_ID = "Link_6_1";
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
    std::map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER,  1.0},
                                            {RIGHT_TRIGGER, 1.0}};
    std::map<Button, double> BUTTON_DEFAULTS;

    // Publisher for the twist command
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    // Publisher for the joint command
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    // Subscriber for the joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Services to interact with Servo
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                         std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                         std::unique_ptr<control_msgs::msg::JointJog>& joint);
public:

    xbox_controller();

    ~xbox_controller();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);



};


#endif //ROBO_TELEOPERATION_XBOXCONTROLLER_HPP
