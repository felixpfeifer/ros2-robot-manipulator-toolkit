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


class SpaceMouse : public rclcpp::Node {
private:

    // We'll just set up parameters here
    const std::string INPUT_TOPIC = "/spacenav/twist";
    const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
    const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
    const std::string EEF_FRAME_ID = "Link_6_1";
    const std::string BASE_FRAME_ID = "world";

    // Twist data of the calibration
    geometry_msgs::msg::Twist calibration_offset_;
    bool calibrated_;
    rclcpp::Time start_calibration_time_;
    geometry_msgs::msg::Twist accumulated_twist;
    int sample_count = 0;

    void calibrateSpaceMouse(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Publisher for the twist command
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    // Subscriber for the Spacenav Twist Command
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr space_mouse_sub;

    // Services to interact with Servo
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

public:

    SpaceMouse();

    ~SpaceMouse();

    void spaceMouseCallback(const geometry_msgs::msg::Twist::SharedPtr msg);



};


#endif //ROBO_TELEOPERATION_XBOXCONTROLLER_HPP
