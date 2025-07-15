#include "toilet_scene.h"

ToiletScene::ToiletScene(rclcpp::Node::SharedPtr node) : node_(node) {}

bool ToiletScene::addObjectWithRetry(const std::string& object_name, std::function<bool()> add_function)
{
    const int MAX_RETRIES = 3;
    for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        RCLCPP_INFO(node_->get_logger(), "Adding %s (attempt %d/%d)...", 
                   object_name.c_str(), attempt, MAX_RETRIES);
        
        if (add_function()) {
            RCLCPP_INFO(node_->get_logger(), "✓ %s added successfully!", object_name.c_str());
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), "✗ Failed to add %s, retrying...", object_name.c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "✗ Failed to add %s after %d attempts!", 
                object_name.c_str(), MAX_RETRIES);
    return false;
}

bool ToiletScene::verifyObjectsAdded(const std::vector<std::string>& expected_objects)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    rclcpp::sleep_for(std::chrono::milliseconds(200));  // Brief wait for scene update
    
    auto existing_objects = psi.getKnownObjectNames();
    
    for (const auto& expected : expected_objects) {
        bool found = std::find(existing_objects.begin(), existing_objects.end(), expected) != existing_objects.end();
        if (!found) {
            RCLCPP_WARN(node_->get_logger(), "Object '%s' not found in scene", expected.c_str());
            return false;
        }
    }
    return true;
}

bool ToiletScene::addMirror(double x, double y, double z)
{
    return addObjectWithRetry("Mirror", [this, x, y, z]() {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        moveit_msgs::msg::CollisionObject mirror;
        mirror.header.frame_id = "world";
        mirror.id = "mirror";
        
        shape_msgs::msg::SolidPrimitive mirror_primitive;
        mirror_primitive.type = mirror_primitive.BOX;
        mirror_primitive.dimensions.resize(3);
        mirror_primitive.dimensions[0] = 0.02; // thickness
        mirror_primitive.dimensions[1] = 0.6;  // width
        mirror_primitive.dimensions[2] = 0.4;  // height
        
        geometry_msgs::msg::Pose mirror_pose;
        mirror_pose.position.x = x;
        mirror_pose.position.y = y;
        mirror_pose.position.z = z;
        mirror_pose.orientation.w = 1.0;
        
        mirror.primitives.push_back(mirror_primitive);
        mirror.primitive_poses.push_back(mirror_pose);
        mirror.operation = mirror.ADD;
        
        planning_scene_interface.addCollisionObjects({mirror});
        
        RCLCPP_INFO(node_->get_logger(), "Mirror positioned at x=%.2f, y=%.2f, z=%.2f", x, y, z);
        
        return verifyObjectsAdded({"mirror"});
    });
}

bool ToiletScene::addNozzleTool(double x, double y, double z)
{
    return addObjectWithRetry("Nozzle Tool", [this, x, y, z]() {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        // Nozzle tip
        moveit_msgs::msg::CollisionObject nozzle_tip;
        nozzle_tip.header.frame_id = "world";
        nozzle_tip.id = "nozzle_tip";
        
        shape_msgs::msg::SolidPrimitive tip_primitive;
        tip_primitive.type = tip_primitive.BOX;
        tip_primitive.dimensions.resize(3);
        tip_primitive.dimensions[0] = NOZZLE_TIP_WIDTH;
        tip_primitive.dimensions[1] = NOZZLE_TIP_DEPTH;
        tip_primitive.dimensions[2] = NOZZLE_TIP_HEIGHT;
        
        geometry_msgs::msg::Pose tip_pose;
        tip_pose.position.x = x;
        tip_pose.position.y = y;
        tip_pose.position.z = z;
        tip_pose.orientation.w = 1.0;
        
        nozzle_tip.primitives.push_back(tip_primitive);
        nozzle_tip.primitive_poses.push_back(tip_pose);
        nozzle_tip.operation = nozzle_tip.ADD;
        
        // Nozzle handle (above tip)
        moveit_msgs::msg::CollisionObject nozzle_handle;
        nozzle_handle.header.frame_id = "world";
        nozzle_handle.id = "nozzle_handle";
        
        shape_msgs::msg::SolidPrimitive handle_primitive;
        handle_primitive.type = handle_primitive.CYLINDER;
        handle_primitive.dimensions.resize(2);
        handle_primitive.dimensions[0] = HANDLE_LENGTH;
        handle_primitive.dimensions[1] = HANDLE_RADIUS;
        
        geometry_msgs::msg::Pose handle_pose;
        handle_pose.position.x = x;
        handle_pose.position.y = y;
        handle_pose.position.z = z + (NOZZLE_TIP_HEIGHT / 2.0) + (HANDLE_LENGTH / 2.0);
        handle_pose.orientation.w = 1.0;
        
        nozzle_handle.primitives.push_back(handle_primitive);
        nozzle_handle.primitive_poses.push_back(handle_pose);
        nozzle_handle.operation = nozzle_handle.ADD;
        
        planning_scene_interface.addCollisionObjects({nozzle_tip, nozzle_handle});
        
        RCLCPP_INFO(node_->get_logger(), "Nozzle tool positioned at x=%.2f, y=%.2f, z=%.2f", x, y, z);
        
        return verifyObjectsAdded({"nozzle_tip", "nozzle_handle"});
    });
}

bool ToiletScene::addBrushTool(double x, double y, double z)
{
    return addObjectWithRetry("Brush Tool", [this, x, y, z]() {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Brush head
        moveit_msgs::msg::CollisionObject brush_head;
        brush_head.header.frame_id = "world";
        brush_head.id = "brush_head";

        shape_msgs::msg::SolidPrimitive head_primitive;
        head_primitive.type = head_primitive.CYLINDER;
        head_primitive.dimensions.resize(2);
        head_primitive.dimensions[0] = BRUSH_HEAD_HEIGHT;
        head_primitive.dimensions[1] = BRUSH_HEAD_RADIUS;

        geometry_msgs::msg::Pose head_pose;
        head_pose.position.x = x;
        head_pose.position.y = y;
        head_pose.position.z = z;
        head_pose.orientation.w = 1.0;

        brush_head.primitives.push_back(head_primitive);
        brush_head.primitive_poses.push_back(head_pose);
        brush_head.operation = brush_head.ADD;

        // Brush handle (above head)
        moveit_msgs::msg::CollisionObject brush_handle;
        brush_handle.header.frame_id = "world";
        brush_handle.id = "brush_handle";

        shape_msgs::msg::SolidPrimitive handle_primitive;
        handle_primitive.type = handle_primitive.CYLINDER;
        handle_primitive.dimensions.resize(2);
        handle_primitive.dimensions[0] = HANDLE_LENGTH;
        handle_primitive.dimensions[1] = HANDLE_RADIUS;

        geometry_msgs::msg::Pose handle_pose;
        handle_pose.position.x = x;
        handle_pose.position.y = y;
        handle_pose.position.z = z + (BRUSH_HEAD_HEIGHT / 2.0) + (HANDLE_LENGTH / 2.0);
        handle_pose.orientation.w = 1.0;

        brush_handle.primitives.push_back(handle_primitive);
        brush_handle.primitive_poses.push_back(handle_pose);
        brush_handle.operation = brush_handle.ADD;

        planning_scene_interface.addCollisionObjects({brush_head, brush_handle});

        RCLCPP_INFO(node_->get_logger(), "Brush tool positioned at x=%.2f, y=%.2f, z=%.2f", x, y, z);

        return verifyObjectsAdded({"brush_head", "brush_handle"});
    });
}

bool ToiletScene::addSink(double x, double y, double z)
{
    return addObjectWithRetry("Sink", [this, x, y, z]() {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        // Sink base
        moveit_msgs::msg::CollisionObject sink_base;
        sink_base.header.frame_id = "world";
        sink_base.id = "sink_base";
        
        shape_msgs::msg::SolidPrimitive base_primitive;
        base_primitive.type = base_primitive.CYLINDER;
        base_primitive.dimensions.resize(2);
        base_primitive.dimensions[0] = TABLE_Z;  // height
        base_primitive.dimensions[1] = 0.05;     // radius
        
        geometry_msgs::msg::Pose base_pose;
        base_pose.position.x = x;
        base_pose.position.y = y;
        base_pose.position.z = z;
        base_pose.orientation.w = 1.0;
        
        sink_base.primitives.push_back(base_primitive);
        sink_base.primitive_poses.push_back(base_pose);
        sink_base.operation = sink_base.ADD;
        
        // Sink bowl
        moveit_msgs::msg::CollisionObject sink_bowl;
        sink_bowl.header.frame_id = "world";
        sink_bowl.id = "sink_bowl";
        
        shape_msgs::msg::SolidPrimitive bowl_primitive;
        bowl_primitive.type = bowl_primitive.CYLINDER;
        bowl_primitive.dimensions.resize(2);
        bowl_primitive.dimensions[0] = 0.1;  // height
        bowl_primitive.dimensions[1] = 0.15; // radius
        
        geometry_msgs::msg::Pose bowl_pose;
        bowl_pose.position.x = x;
        bowl_pose.position.y = y;
        bowl_pose.position.z = z + TABLE_Z + 0.05;  // On top of base
        bowl_pose.orientation.w = 1.0;
        
        sink_bowl.primitives.push_back(bowl_primitive);
        sink_bowl.primitive_poses.push_back(bowl_pose);
        sink_bowl.operation = sink_bowl.ADD;
        
        planning_scene_interface.addCollisionObjects({sink_base, sink_bowl});
        
        RCLCPP_INFO(node_->get_logger(), "Sink positioned at x=%.2f, y=%.2f, z=%.2f", x, y, z);
        
        return verifyObjectsAdded({"sink_base", "sink_bowl"});
    });
}