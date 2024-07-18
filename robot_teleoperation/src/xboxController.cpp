#include "robot_teleoperation/xboxController.hpp"
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;

xbox_controller::xbox_controller() : Node("xbox_controller") {
    // Set up the publisher for the twist command
    node = std::make_shared<rclcpp::Node>("xbox_internal_node");

    twist_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
    RCLCPP_INFO(this->get_logger(), "Twist publisher initialized");

    // Set up the publisher for the joint command
    joint_pub_ = node->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);
    RCLCPP_INFO(this->get_logger(), "Joint publisher initialized");

    // Set up the subscriber for the joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, 10,
                                                                std::bind(&xbox_controller::joy_callback, this,
                                                                          std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Joystick subscriber initialized");

    // Service Client for the Teleop Backend
    // Select Tool TCP for the Teleop Backend
    select_tool_client = node->create_client<robot_teleoperation_interface::srv::SelectTool>("/select_tool");
    RCLCPP_INFO(this->get_logger(), "Waiting for /select_tool service...");
    select_tool_client->wait_for_service(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "Service /select_tool available");
    // Open Close the Gripper Tool
    tool_client = node->create_client<robot_teleoperation_interface::srv::Tool>("/tool");
    RCLCPP_INFO(this->get_logger(), "Waiting for /tool service...");
    tool_client->wait_for_service(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "Service /tool available");


    // Create a service client to start the ServoNode
    servo_start_client_ = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo service...");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "Service /servo_node/start_servo available");

    servo_stop_client_ = node->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "Service request to start servo node sent");

    // Services to get and set the parameters of the Servo Node
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(node, "servo_node");
}

void xbox_controller::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Joystick callback triggered");

    // Create the twist and joint messages
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Convert the joystick message to a twist or joint message
    bool is_twist = convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg);
    if (is_twist) {
        // publish the TwistStamped
        twist_msg->header.frame_id = BASE_FRAME_ID;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
        //RCLCPP_INFO(this->get_logger(), "Published TwistStamped message");
    }
}

xbox_controller::~xbox_controller() {
    RCLCPP_INFO(this->get_logger(), "xbox_controller node shutting down");
}

bool xbox_controller::convertJoyToCmd(const std::vector<float> &axes, const std::vector<int> &buttons,
                                      std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist,
                                      std::unique_ptr<control_msgs::msg::JointJog> &joint) {
    //RCLCPP_INFO(this->get_logger(), "convertJoyToCmd called");

    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    // Buttons A is to open/close the Gripper
    // If the Gripper is open, close it
    // Use debounding for the button
    if (buttons[A] && debouncing_states["A"] == 0) {
        RCLCPP_INFO(this->get_logger(), "Button A pressed");
        debouncing_states["A"] = 1;
        is_gripper_open = !is_gripper_open;
        auto request = std::make_shared<robot_teleoperation_interface::srv::Tool::Request>();
        request->close = is_gripper_open;
        auto response = tool_client->async_send_request(request);
        rclcpp::spin_until_future_complete(node, response);
        if (response.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Gripper is %s", is_gripper_open ? "opened" : "closed");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open/close the gripper");
        }
    } else if (buttons[A] == 0 && debouncing_states["A"] == 1) {
        debouncing_states["A"] = 0;
    }

    // Button B is to change the Parameter planning_frame to world or to TCP_Gripper
    // And Restart the Servo node to accept the new parameter
    if (buttons[B] && debouncing_states["B"] == 0) {
        // Get the current Planning Frame
        RCLCPP_INFO(this->get_logger(), "Button B pressed");
        debouncing_states["B"] = 1;
        // Set a timeout duration
        std::chrono::seconds timeout(5);

        // Ensure parameter client is initialized
        if (!parameter_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Parameter service is not available");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Parameter service is available");

        // Get the current value of the planning_frame parameter
        auto parameters = parameter_client_->get_parameters({"moveit_servo.planning_frame"});
        if (parameters.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get parameter");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Current planning_frame: %s", parameters[0].get_parameter_value().to_value_msg().string_value.c_str());
        // Change the value of the parameter between "world" and "tcp_gripper"
        std::string new_value = parameters[0].as_string() == "world" ? "tcp_gripper" : "world";
        RCLCPP_INFO(this->get_logger(), "Setting new value: %s", new_value.c_str());

        // Set the new value of the parameter
        rclcpp::Parameter new_parameter("moveit_servo.planning_frame", new_value);
        auto set_parameters_results = parameter_client_->set_parameters({new_parameter});

        for (auto &result: set_parameters_results) {
            if (!result.successful) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully set parameter.");
            }
        }

        // Restart the Servo Node
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto response = servo_stop_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(node, response);
        if (response.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Servo node stopped");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop servo node");
        }
        // Start the Servo Node
        response = servo_start_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(node, response);
        if (response.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Servo node started");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start servo node");
        }
        

    } else if (buttons[B] == 0 && debouncing_states["B"] == 1) {
        debouncing_states["B"] = 0;
    }

    // Button X is to select the Link_6_1
    if (buttons[X] && debouncing_states["X"] == 0) {

    } else if (buttons[X] == 0 && debouncing_states["X"] == 1) {
        debouncing_states["X"] = 0;
    }

    // Button Y to select the camera_tcp link
    if (buttons[Y] && debouncing_states["Y"] == 0) {

    } else if (buttons[Y] == 0 && debouncing_states["Y"] == 1) {
        debouncing_states["Y"] = 0;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = -axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;

    // Max Twist Velocities in xyz is 0.25 m / s enfore this

    double vel = sqrt(pow(twist->twist.linear.x, 2) + pow(twist->twist.linear.y, 2) + pow(twist->twist.linear.z, 2));
    // If the velocity is greater than the max, scale it down
    if (vel > max_vel) {
        twist->twist.linear.x = twist->twist.linear.x / vel * max_vel;
        twist->twist.linear.y = twist->twist.linear.y / vel * max_vel;
        twist->twist.linear.z = twist->twist.linear.z / vel * max_vel;
    }
    // Same for the Rotation with 0.5 rad/s

    double rot = sqrt(pow(twist->twist.angular.x, 2) + pow(twist->twist.angular.y, 2) + pow(twist->twist.angular.z, 2));
    if (rot > max_rot) {
        twist->twist.angular.x = twist->twist.angular.x / rot * max_rot;
        twist->twist.angular.y = twist->twist.angular.y / rot * max_rot;
        twist->twist.angular.z = twist->twist.angular.z / rot * max_rot;
    }

    return true;
}

void xbox_controller::tool_callback(rclcpp::Client<robot_teleoperation_interface::srv::Tool>::SharedFuture future) {

    // On Success the gripper is opened or closed and the state is updated
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Gripper is %s", is_gripper_open ? "opened" : "closed");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open/close the gripper");
    }
}

void xbox_controller::select_tool_callback(
        rclcpp::Client<robot_teleoperation_interface::srv::SelectTool>::SharedFuture future) {

    // On Success the tool is selected and the state is updated
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Tool selected");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to select the tool");
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<xbox_controller>());
    rclcpp::shutdown();
    return 0;
}
