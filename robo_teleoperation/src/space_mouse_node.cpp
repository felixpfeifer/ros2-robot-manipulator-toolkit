#include <rclcpp/executors.hpp>
#include "../include/robo_teleoperation/space_mouse_node.hpp"

SpaceMouse::SpaceMouse() : Node("space_mouse_node"), calibrated_(false) {
    space_mouse_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spacenav/twist", 10, std::bind(&SpaceMouse::spaceMouseCallback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo service...");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "Service /servo_node/start_servo available");

    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "Service request to start servo node sent");

    // Start calibration
    RCLCPP_INFO(this->get_logger(), "Calibrating space mouse, please keep it stationary...");
    calibrated_ = false;
    start_calibration_time_ = this->now();

}

SpaceMouse::~SpaceMouse() {
}

void SpaceMouse::spaceMouseCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    calibrated_ = true;
    if (!calibrated_) {

        calibrateSpaceMouse(msg);
        return;
    }

    auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist->header.frame_id = BASE_FRAME_ID;
    twist->header.stamp = this->now();

    // Apply calibration offset
    twist->twist.linear.x = msg->linear.x - calibration_offset_.linear.x;
    twist->twist.linear.y = msg->linear.y - calibration_offset_.linear.y;
    twist->twist.linear.z = msg->linear.z - calibration_offset_.linear.z;
    twist->twist.angular.x = msg->angular.x - calibration_offset_.angular.x;
    twist->twist.angular.y = msg->angular.y - calibration_offset_.angular.y;
    twist->twist.angular.z = msg->angular.z - calibration_offset_.angular.z;



    // Scale down the twist values if they exceed the limits
    double max_vel = 0.25;
    double vel = sqrt(pow(twist->twist.linear.x, 2) + pow(twist->twist.linear.y, 2) + pow(twist->twist.linear.z, 2));
    if (vel > max_vel) {
        RCLCPP_INFO(this->get_logger(), "Linear velocity exceeded, scaling down");
        twist->twist.linear.x = twist->twist.linear.x / vel * max_vel;
        twist->twist.linear.y = twist->twist.linear.y / vel * max_vel;
        twist->twist.linear.z = twist->twist.linear.z / vel * max_vel;
    }

    double max_rot = 1;
    double rot = sqrt(pow(twist->twist.angular.x, 2) + pow(twist->twist.angular.y, 2) + pow(twist->twist.angular.z, 2));
    if (rot > max_rot) {
        RCLCPP_INFO(this->get_logger(), "Rotational velocity exceeded, scaling down");
        twist->twist.angular.x = twist->twist.angular.x / rot * max_rot;
        twist->twist.angular.y = twist->twist.angular.y / rot * max_rot;
        twist->twist.angular.z = twist->twist.angular.z / rot * max_rot;
    }

    twist_pub_->publish(std::move(twist));
    RCLCPP_INFO(this->get_logger(), "Published TwistStamped message");
}

void SpaceMouse::calibrateSpaceMouse(geometry_msgs::msg::Twist::SharedPtr msg) {

    // If the calibration time is within 10 seconds, accumulate the twist values
    if ((this->now() - start_calibration_time_).seconds() < 15) {

        if (msg) {
            accumulated_twist.linear.x += msg->linear.x;
            accumulated_twist.linear.y += msg->linear.y;
            accumulated_twist.linear.z += msg->linear.z;
            accumulated_twist.angular.x += msg->angular.x;
            accumulated_twist.angular.y += msg->angular.y;
            accumulated_twist.angular.z += msg->angular.z;
            sample_count++;
        }

        rclcpp::Rate(10).sleep();  // Sleep for 100ms
    } else {
        calibration_offset_.linear.x = accumulated_twist.linear.x / sample_count;
        calibration_offset_.linear.y = accumulated_twist.linear.y / sample_count;
        calibration_offset_.linear.z = accumulated_twist.linear.z / sample_count;
        calibration_offset_.angular.x = accumulated_twist.angular.x / sample_count;
        calibration_offset_.angular.y = accumulated_twist.angular.y / sample_count;
        calibration_offset_.angular.z = accumulated_twist.angular.z / sample_count;
        calibrated_ = true;
        RCLCPP_INFO(this->get_logger(), "Calibration complete. Offset values: "
                                        "Linear: [%f, %f, %f], Angular: [%f, %f, %f]",
                    calibration_offset_.linear.x, calibration_offset_.linear.y, calibration_offset_.linear.z,
                    calibration_offset_.angular.x, calibration_offset_.angular.y, calibration_offset_.angular.z);
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpaceMouse>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
