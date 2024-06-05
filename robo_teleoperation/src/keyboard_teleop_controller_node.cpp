#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rclcpp/rclcpp.hpp>

#include "iostream"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("keyboard_teleop_controller_node");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("keyboard_teleop_controller_node", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    static const std::string PLANNING_GROUP = "robot";


    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Print Joint States
    // Get the current state
    auto current_state = move_group.getCurrentState();

    // Get the joint model group
    const auto joint_model_group_ = current_state->getJointModelGroup(PLANNING_GROUP);

    // Get the joint values
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group_, joint_values);

    // Print the joint values
    for (std::size_t i = 0; i < joint_values.size(); ++i)
    {
        RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_model_group_->getVariableNames()[i].c_str(), joint_values[i]);
    }

    while(rclcpp::ok()){
        // Get the joint values
        current_state->copyJointGroupPositions(joint_model_group_, joint_values);

        // Print the joint values
        for (std::size_t i = 0; i < joint_values.size(); ++i)
        {
            RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_model_group_->getVariableNames()[i].c_str(), joint_values[i]);
        }

        // Input the joint number and desired joint value in degrees
        int joint_number;
        double joint_value;
        RCLCPP_INFO(LOGGER, "Enter the joint number and desired joint value in degrees: ");
        std::cin >> joint_number >> joint_value;
        // Transform the joint value from degrees to radians
        joint_value = joint_value * M_PI / 180.0;
        // Set the joint value
        joint_values[joint_number] = joint_value;
        // Set the joint values
        move_group.setJointValueTarget(joint_values);
        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group.plan(my_plan);
        // Execute the trajectory
        auto error = move_group.execute(my_plan);
        if (error == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Success");
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Error");
        }

    }



    rclcpp::shutdown();
    return 0;
}