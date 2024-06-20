#include <rclcpp/rclcpp.hpp>
#include "robo_teleoperation/xboxController.hpp"

xbox_controller::xbox_controller() : Node("xbox_controller") {
    // Set up the publisher for the twist command
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
    RCLCPP_INFO(this->get_logger(), "Twist publisher initialized");

    // Set up the publisher for the joint command
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);
    RCLCPP_INFO(this->get_logger(), "Joint publisher initialized");

    // Set up the subscriber for the joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, 10,
                                                                std::bind(&xbox_controller::joy_callback, this,
                                                                          std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Joystick subscriber initialized");

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo service...");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "Service /servo_node/start_servo available");

    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "Service request to start servo node sent");
}

void xbox_controller::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Joystick callback triggered");

    // Create the twist and joint messages
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Convert the joystick message to a twist or joint message
    bool is_twist = convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg);
    if (is_twist){
        // publish the TwistStamped
        twist_msg->header.frame_id = BASE_FRAME_ID;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
        RCLCPP_INFO(this->get_logger(), "Published TwistStamped message");
    } else {
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = EEF_FRAME_ID;
        joint_pub_->publish(std::move(joint_msg));
        RCLCPP_INFO(this->get_logger(), "Published JointJog message");
    }
}

xbox_controller::~xbox_controller() {
    RCLCPP_INFO(this->get_logger(), "xbox_controller node shutting down");
}

bool xbox_controller::convertJoyToCmd(const std::vector<float> &axes, const std::vector<int> &buttons,
                                      std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist,
                                      std::unique_ptr<control_msgs::msg::JointJog> &joint) {
    RCLCPP_INFO(this->get_logger(), "convertJoyToCmd called");

    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y]) {
        RCLCPP_INFO(this->get_logger(), "Joint jog command detected");
        
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("joint_1");
        joint->velocities.push_back(axes[D_PAD_X]);
        joint->joint_names.push_back("joint_2");
        joint->velocities.push_back(axes[D_PAD_Y]);

        RCLCPP_INFO(this->get_logger(), "D_PAD mapped to joint_1 and joint_2");

        // Map the diamond to the distal joints
        joint->joint_names.push_back("joint_6");
        joint->velocities.push_back(buttons[B] - buttons[X]);
        joint->joint_names.push_back("joint_5");
        joint->velocities.push_back(buttons[Y] - buttons[A]);

        RCLCPP_INFO(this->get_logger(), "Diamond buttons mapped to joint_5 and joint_6");

        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;

    // Max Twist Velocities in xyz is 0.25 m / s enfore this
    double max_vel = 0.25;
    double vel = sqrt(pow(twist->twist.linear.x, 2) + pow(twist->twist.linear.y, 2) + pow(twist->twist.linear.z, 2));
    // If the velocity is greater than the max, scale it down
    if (vel > max_vel) {
        twist->twist.linear.x = twist->twist.linear.x / vel * max_vel;
        twist->twist.linear.y = twist->twist.linear.y / vel * max_vel;
        twist->twist.linear.z = twist->twist.linear.z / vel * max_vel;
    }
    // Same for the Rotation with 0.5 rad/s
    double max_rot = 0.5;
    double rot = sqrt(pow(twist->twist.angular.x, 2) + pow(twist->twist.angular.y, 2) + pow(twist->twist.angular.z, 2));
    if (rot > max_rot) {
        twist->twist.angular.x = twist->twist.angular.x / rot * max_rot;
        twist->twist.angular.y = twist->twist.angular.y / rot * max_rot;
        twist->twist.angular.z = twist->twist.angular.z / rot * max_rot;
    }

    RCLCPP_INFO(this->get_logger(), "Twist command created: linear.x = %f, linear.y = %f, linear.z = %f, angular.x = %f, angular.y = %f, angular.z = %f",
                 twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z,
                 twist->twist.angular.x, twist->twist.angular.y, twist->twist.angular.z);


    return true;
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<xbox_controller>());
    rclcpp::shutdown();
    return 0;
}
