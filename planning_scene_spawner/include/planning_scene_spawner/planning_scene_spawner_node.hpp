//
// Created by felix on 04.07.24.
//

#ifndef PLANNING_SCENE_SPAWNER_PLANNINGSCENESPAWNERNODE_HPP
#define PLANNING_SCENE_SPAWNER_PLANNINGSCENESPAWNERNODE_HPP

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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>


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


class planningSceneSpawnerNode : public rclcpp::Node {

public:
    planningSceneSpawnerNode();

    ~planningSceneSpawnerNode();

    /*
     * Function to create the Planning Scene for the robot
     * Adds the table and the robot to the planning scene
     *
     */
    void setupPlanningScene();

private:

    // Parameters
    std::string mongo_uri;
    std::string mongo_db;
    std::string mongo_collection;

    // MongoDB
    mongocxx::instance instance{};
    mongocxx::client client;
    mongocxx::database db;
    mongocxx::collection collection;


    // Moveit
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface *moveGroupInterface_;
    moveit_msgs::msg::PlanningScene planning_scene;
    // Planning Scene
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;

    std::string planning_frame_;

    // Helper functions
    // Collision Objects in the Scene
    void addBox();

    void addPLT();

    void addCalibrationPlate();

    /**
     * Function to get the Pose of the Object from the given Points
     * @param id : The ID of the Object
     * @param points : The Points of the Object Corners
     * @param object : The Moveit Object
     * @return The Pose of the Object
     */
    geometry_msgs::msg::Pose getObjectPose(int id, shapes::Mesh *mesh);

    // Function to get the corners Point of the Objects nearest to the robot
    std::vector<geometry_msgs::msg::Point> getColisionCorners(int id);

    // Function to get the size and width of a STL File for the Collision Object
    std::vector<double> getSTLSize(shapes::Mesh *mesh);


};


#endif //PLANNING_SCENE_SPAWNER_PLANNINGSCENESPAWNERNODE_HPP
