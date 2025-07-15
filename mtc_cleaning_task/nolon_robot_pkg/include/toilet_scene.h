#ifndef TOILET_SCENE_H
#define TOILET_SCENE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <algorithm>
#include <functional>

class ToiletScene
{
public:
    ToiletScene(rclcpp::Node::SharedPtr node);
    bool addMirror(double x, double y, double z);
    bool addNozzleTool(double x, double y, double z);
    bool addBrushTool(double x, double y, double z);
    bool addSink(double x, double y, double z);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Object dimensions (kept internal)
    const double HANDLE_LENGTH = 0.1;
    const double HANDLE_RADIUS = 0.01;
    const double NOZZLE_TIP_WIDTH = 0.04;
    const double NOZZLE_TIP_DEPTH = 0.04;
    const double NOZZLE_TIP_HEIGHT = 0.05;
    const double BRUSH_HEAD_HEIGHT = 0.1;
    const double BRUSH_HEAD_RADIUS = 0.05;
    const double TABLE_Z = 0.05;

    bool addObjectWithRetry(const std::string& object_name, std::function<bool()> add_function);
    bool verifyObjectsAdded(const std::vector<std::string>& expected_objects);
};

#endif // TOILET_SCENE_H