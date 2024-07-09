//
// Created by felix on 04.07.24.
//

#include "../include/planning_scene_spawner/planning_scene_spawner_node.hpp"

void planningSceneSpawnerNode::setupPlanningScene() {
    // Add a Collision Plane under the Robot to avoid collision with the ground
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = "ground_plane";

    // Define the plane dimensions (in meters)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 10.0;// length
    primitive.dimensions[1] = 10.0;// width
    primitive.dimensions[2] = 0.01;// height

    // Define the pose of the plane (relative to the planning frame)
    geometry_msgs::msg::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.z = -0.005;// half of the height

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(plane_pose);
    collision_object.operation = collision_object.ADD;
    // Add to the planning scene
    planning_scene.world.collision_objects.push_back(collision_object);
    RCLCPP_INFO(this->get_logger(), "Added Ground Plane to Planning Scene");
    addPLT();
    RCLCPP_INFO(this->get_logger(), "Added PLT to Planning Scene");
    addBox();
    RCLCPP_INFO(this->get_logger(), "Added Box to Planning Scene");
    addCalibrationPlate();
    RCLCPP_INFO(this->get_logger(), "Added Calibration Plate to Planning Scene");
    // Update the Planning Scene
    planning_scene.is_diff = true;
    planning_scene_publisher_->publish(planning_scene);
}

void planningSceneSpawnerNode::addPLT() {
    // object_id = 1
    // Add the Calibration Plate to the Planning Scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = "plt";
    shapes::Shape *plt = shapes::createMeshFromResource(
            "package://robo_teleoperation/meshes/palette.stl");
    RCLCPP_INFO(this->get_logger(), "Loaded Mesh");
    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(plt, mesh_msg);
    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    collision_object.meshes.push_back(mesh);
    geometry_msgs::msg::Pose plt_pose;
    shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
    plt_pose = getObjectPose(1, meshet);
    RCLCPP_INFO(this->get_logger(), "Got Pose");
    plt_pose.position.z = plt_pose.position.z + 0.015;
    collision_object.mesh_poses.push_back(plt_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);
}

void planningSceneSpawnerNode::addBox() {
    // object_id = 2
    // Add the Box to the Planning Scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = "kiste";
    shapes::Shape *plt = shapes::createMeshFromResource(
            "package://robo_teleoperation/meshes/kiste.stl");

    shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(plt, mesh_msg);
    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    collision_object.meshes.push_back(mesh);
    geometry_msgs::msg::Pose plt_pose;
    plt_pose = getObjectPose(3, meshet);
    collision_object.mesh_poses.push_back(plt_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);
}

void planningSceneSpawnerNode::addCalibrationPlate() {
    // object_id = 2
    // Add the Plate to the Planning Scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = "platte";
    shapes::Shape *plt = shapes::createMeshFromResource(
            "package://robo_teleoperation/meshes/kalibierplatte.stl");
    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(plt, mesh_msg);
    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    collision_object.meshes.push_back(mesh);
    geometry_msgs::msg::Pose plt_pose;
    shapes::Mesh *meshet = static_cast<shapes::Mesh *>(plt);
    plt_pose = getObjectPose(2, meshet);
    collision_object.mesh_poses.push_back(plt_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);
}


std::vector<geometry_msgs::msg::Point> planningSceneSpawnerNode::getColisionCorners(int id) {
    std::vector<geometry_msgs::msg::Point> point_list;
    std::vector<int> order;
    // resize the vector to 2
    // Load the Collision Object from the MongoDB DB with collection collection
    // Withh the colision "object_id"
    auto filter = bsoncxx::builder::stream::document{} << "object_id" << id
                                                       << bsoncxx::builder::stream::finalize;
    RCLCPP_INFO(this->get_logger(), "Get Collision Object with ID: %d", id);
    // Find all Points of the Collision Object
    auto result = collection.find(filter.view());
    RCLCPP_INFO(this->get_logger(), "Found Collision Object with ID: %d", id);
    // Check if the cursor is not empty and process the results
    for (auto &&doc: result) {
        // Parse the BSON document to extract the points
        // In the Document is a var point_number with the number of points
        // Get the number of the point
        int point_number = doc["point_number"].get_int32();

        if (doc["points"]) {
            auto points = doc["points"].get_array().value;
            double x = points[0].get_double();
            double y = points[1].get_double();
            double z = points[2].get_double();
            // Convert to a Point Message
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            point_list.push_back(p);
            order.push_back(point_number);
            // Do something with the point (e.g., log or use in collision object)
            RCLCPP_INFO(this->get_logger(), "Point position: [x: %f, y: %f, z: %f]", x, y, z);
        }
    }
    // Sort the Points after the id in the order vector
    std::vector<geometry_msgs::msg::Point> sorted_points;
    if (order[0] < order[1]) {
        sorted_points.push_back(point_list[0]);
        sorted_points.push_back(point_list[1]);
    } else {
        sorted_points.push_back(point_list[1]);
        sorted_points.push_back(point_list[0]);
    }
    return sorted_points;
}

geometry_msgs::msg::Pose
planningSceneSpawnerNode::getObjectPose(int id, shapes::Mesh *mesh) {

    // Pose of the Object
    geometry_msgs::msg::Pose box_center_pose;

    // Size of the Object
    std::vector<double> size = getSTLSize(mesh);
    double length = size[0];
    double width = size[1];
    double height = size[2];


    // Get the two connected corners of the object
    std::vector<geometry_msgs::msg::Point> corners = getColisionCorners(id);

    // Ensure we have exactly two points
    if (corners.size() != 2) {
        throw std::runtime_error("Expected exactly two corner points");
    }

    double dx = corners[1].x - corners[0].x;
    double dy = corners[1].y - corners[0].y;
    double distance_between_corners = sqrt(dx * dx + dy * dy);

    double edge_length = 0;
    double other_edge_length = 0;
    // Determine if the given edge is the width or the length of the object
    if (abs(distance_between_corners - width) < abs(distance_between_corners - length)) {
        edge_length = width;
        other_edge_length = length;
    } else {
        edge_length = length;
        other_edge_length = width;
    }
    // Calulate the orientation of the object with angel theta
    double theta = atan2(dy, dx);
    // Calculate the normal vector of the given edge with the eigen lib
    Eigen::Vector2d normal_vector(-dy, dx);
    normal_vector = normal_vector / distance_between_corners;
    // Calculate the center of the object
    Eigen::Vector2d center_offset = (other_edge_length / 2) * normal_vector;
    auto x_center = (corners[0].x + corners[1].x) / 2 + center_offset[0];
    auto y_center = (corners[0].y + corners[1].y) / 2 + center_offset[1];

    // Calculate the half-dimensions
    double half_edge = edge_length / 2;
    double half_other_edge = other_edge_length / 2;

    // Set the position of the object
    box_center_pose.position.x = x_center;
    box_center_pose.position.y = y_center;
    box_center_pose.position.z = 0;// Assume the object is on the ground

    // Set the orientation of the object
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    tf2::convert(q, box_center_pose.orientation);

    return box_center_pose;
}

std::vector<double> planningSceneSpawnerNode::getSTLSize(shapes::Mesh *mesh) {
    std::vector<double> size;

    // Extract the dimensions of the STL model
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();

    for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
        const double *vertex = &mesh->vertices[3 * i];
        if (vertex[0] < min_x) min_x = vertex[0];
        if (vertex[0] > max_x) max_x = vertex[0];
        if (vertex[1] < min_y) min_y = vertex[1];
        if (vertex[1] > max_y) max_y = vertex[1];
        if (vertex[2] < min_z) min_z = vertex[2];
        if (vertex[2] > max_z) max_z = vertex[2];
    }

    double length = max_x - min_x;
    double width = max_y - min_y;
    double height = max_z - min_z;

    size.push_back(length);
    size.push_back(width);
    size.push_back(height);

    return size;
}

planningSceneSpawnerNode::planningSceneSpawnerNode() : Node("planning_scene_spawner") {

    // Declare the Parameters
    this->declare_parameter<std::string>("mongo.url", "mongodb://localhost:27017");
    this->declare_parameter<std::string>("mongo.database", "default_db");
    this->declare_parameter<std::string>("mongo.collection", {"default_collection"});
    this->declare_parameter<std::string>("planning_frame", "world");

    // Get the Parameters
    mongo_uri = this->get_parameter("mongo.url").as_string();
    mongo_db = this->get_parameter("mongo.database").as_string();
    mongo_collection = this->get_parameter("mongo.collection").as_string();

    planning_frame_ = this->get_parameter("planning_frame").as_string();

    // Create the MongoDB Client
    client = mongocxx::client(mongocxx::uri(mongo_uri));
    db = client[mongo_db];
    collection = db[mongo_collection];

    // Debug print the parameters
    RCLCPP_INFO(this->get_logger(), "MongoDB URI: %s", mongo_uri.c_str());
    RCLCPP_INFO(this->get_logger(), "MongoDB Database: %s", mongo_db.c_str());
    RCLCPP_INFO(this->get_logger(), "MongoDB Collection: %s", mongo_collection.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", planning_frame_.c_str());


    // Create the Planning Scene Interface
    planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
    planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "planning_scene", 10);

}

planningSceneSpawnerNode::~planningSceneSpawnerNode() {

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planningSceneSpawnerNode>();
    node->setupPlanningScene();
    rclcpp::shutdown();
    return 0;
}