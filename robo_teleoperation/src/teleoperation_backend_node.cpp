
#include "robo_teleoperation/teleoperation_backend_node.hpp"

namespace robo_teleoperation {
    using namespace std::chrono_literals;

    TeleoperationBackendNode::TeleoperationBackendNode() : rclcpp::Node("TeleOpBackendNode") {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node = rclcpp::Node::make_shared("move_group_node", node_options);

        executor_.add_node(node);
        std::thread([&]() { executor_.spin(); }).detach();

        moveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

        // Timer that runs every 500ms
        timer_ = this->create_wall_timer(
                20ms, std::bind(&TeleoperationBackendNode::timer_callback, this));

    }


    TeleoperationBackendNode::~TeleoperationBackendNode() {

    }

    void TeleoperationBackendNode::timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer Callback()");

        // Get the robot state
        moveit::core::RobotStatePtr robot_state_ptr = moveGroupInterface->getCurrentState();

        // Get the current Joint Position
        const auto joint_model = robot_state_ptr->getJointModelGroup(PLANNING_GROUP);
        std::vector<double> joint_values;
        robot_state_ptr->copyJointGroupPositions(joint_model, joint_values);

        // Print Current States for the Robot
        for (std::size_t i = 0; i < joint_values.size(); ++i)
        {
            RCLCPP_INFO(logger, "Joint %s: %f", joint_model->getVariableNames()[i].c_str(), joint_values[i]);
        }

    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<robo_teleoperation::TeleoperationBackendNode>());
    rclcpp::shutdown();
    return 0;
}
