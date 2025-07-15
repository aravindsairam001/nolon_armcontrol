#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

#include "toilet_scene.h"

// Position parameters -
const double MIRROR_X = 0.7;
const double MIRROR_Y = 0.0;
const double MIRROR_Z = .75;

const double NOZZLE_X = 0.35;
const double NOZZLE_Y = 0.2;
const double NOZZLE_Z = 0.025;

const double BRUSH_X = 0.35;
const double BRUSH_Y = -0.2;
const double BRUSH_Z = 0.025;

// Parameters
const double SPRAY_DISTANCE = 0.4;   // 25cm-40cm from mirror..collision be ware
const double SPRAY_PAUSE_TIME = 2.0; // 

const double SWIPE_DISTANCE = 0.4;       // 25-40cm from mirror..collision maybe ther for too close
const double SWIPESTEP_PAUSE_TIME = 0.1; // seconds

// Object dimensions (for refernce only, donot change)
const double HANDLE_LENGTH = 0.1;
const double HANDLE_RADIUS = 0.01;
const double NOZZLE_TIP_WIDTH = 0.04;
const double NOZZLE_TIP_DEPTH = 0.04;
const double NOZZLE_TIP_HEIGHT = 0.05;
const double BRUSH_HEAD_HEIGHT = 0.1;
const double BRUSH_HEAD_RADIUS = 0.05;
const double TABLE_Z = 0.05;

class MTCSprayTaskNode
{
public:
  MTCSprayTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doNozzleTask();
  void doBrushTask();
  void setupPlanningScene();
  void planAllTasks();
  void executeAllTasks();
  // bool querySceneObjects();

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  // Compose an MTC task 
  mtc::Task createNozzleTask();
  mtc::Task createBrushTask();
  mtc::Task nozzle_task_;
  mtc::Task brush_task_;

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  // Object poses (will be populated from scene)
  geometry_msgs::msg::Pose nozzle_handle_pose_;
  geometry_msgs::msg::Pose brush_handle_pose_;

  geometry_msgs::msg::Pose mirror_pose_;
  double mirror_width_, mirror_height_;

  //   moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<geometry_msgs::msg::Pose> calculateCleaningPaths()
  {
    std::vector<geometry_msgs::msg::Pose> cleaning_poses;

    // Mirror parameters (from your constants)
    double mirror_x = MIRROR_X;
    double mirror_y = MIRROR_Y;
    double mirror_z = MIRROR_Z;
    double cleaning_distance = 0.3; // 30cm from mirror

    // Cleaning area dimensions..play with this
    double clean_width = mirror_width_ * 0.8;   // Clean 80% of mirror width
    double clean_height = mirror_height_ * 0.8; // Clean 80% of mirror height
    double swipe_spacing = 0.1;                 // 8cm between horizontal swipes

    // Calculate cleaning positions
    double start_x = mirror_x - cleaning_distance;
    double start_y = mirror_y - (clean_width / 2.0);
    double end_y = mirror_y + (clean_width / 2.0);
    double top_z = mirror_z + (clean_height / 2.0);
    double bottom_z = mirror_z - (clean_height / 2.0);

    // Generate horizontal swipe waypoints (top to bottom)
    bool left_to_right = true;
    for (double z = top_z; z >= bottom_z; z -= swipe_spacing)
    {

      // Start point of swipe
      geometry_msgs::msg::Pose start_pose;
      start_pose.position.x = start_x;
      start_pose.position.y = left_to_right ? start_y : end_y;
      start_pose.position.z = z;
      // Brush pointing toward mirror
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.707;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.707;
      cleaning_poses.push_back(start_pose);

      // End point of swipe
      geometry_msgs::msg::Pose end_pose = start_pose;
      end_pose.position.y = left_to_right ? end_y : start_y;
      cleaning_poses.push_back(end_pose);

      left_to_right = !left_to_right; // Alternate direction
    }

    RCLCPP_INFO(LOGGER, "Generated %zu cleaning waypoints", cleaning_poses.size());
    return cleaning_poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculateMirrorCorners()
  {
    std::vector<geometry_msgs::msg::Pose> corners;
    // Mirror at x=0.50, y=-0.30 (but your code shows TARGET_X+0.2, so actual x should be 0.7)
    mirror_pose_.position.x = MIRROR_X;
    mirror_pose_.position.y = MIRROR_Y;
    mirror_pose_.position.z = MIRROR_Z;
    mirror_pose_.orientation.w = 1.0;

    // Mirror dimensions from your toilet_scene.cpp
    mirror_width_ = 0.6;  // width (y dimension)
    mirror_height_ = 0.4; // height (z dimension)

    // Calculate spray positions in front of mirror corners
    double spray_x = mirror_pose_.position.x - SPRAY_DISTANCE; // In front of mirror
    double center_y = mirror_pose_.position.y;
    double center_z = mirror_pose_.position.z;

    // Corner offsets relative to mirror center
    double half_width = mirror_width_ / 2.0;
    double half_height = mirror_height_ / 2.0;

    // Four corners: top-left, top-right, bottom-right, bottom-left
    std::vector<std::pair<double, double>> offsets = {
        {-half_width, half_height}, // top-left
        {half_width, half_height},  // top-right
        {half_width, -half_height}, // bottom-right
        {-half_width, -half_height} // bottom-left
    };

    std::vector<std::string> corner_names = {"top-left", "top-right", "bottom-right", "bottom-left"};

    for (size_t i = 0; i < offsets.size(); ++i)
    {
      const auto &offset = offsets[i];
      geometry_msgs::msg::Pose corner_pose;
      corner_pose.position.x = spray_x;
      corner_pose.position.y = center_y + offset.first;
      corner_pose.position.z = center_z + offset.second;

      corners.push_back(corner_pose);

      // Log each corner position
      RCLCPP_INFO(LOGGER, "Mirror corner %zu (%s): x=%.3f, y=%.3f, z=%.3f",
                  i + 1, corner_names[i].c_str(),
                  corner_pose.position.x, corner_pose.position.y, corner_pose.position.z);
    }

    return corners;
  }
};

MTCSprayTaskNode::MTCSprayTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
  // Initialize move group first
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "panda_arm");

  // Construct and initialize MoveItVisualTools 
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_->getRobotModel());

  // /rviz_visual_tools topic
  visual_tools_->deleteAllMarkers();
  visual_tools_->loadRemoteControl();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCSprayTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCSprayTaskNode::setupPlanningScene()
{
  RCLCPP_INFO(LOGGER, "Setting up toilet scene with objects...");

  // Create toilet scene and add objects at specified positions
  ToiletScene scene(node_);

  if (!scene.addMirror(MIRROR_X, MIRROR_Y, MIRROR_Z))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add mirror to scene!");
    return;
  }

  if (!scene.addNozzleTool(NOZZLE_X, NOZZLE_Y, NOZZLE_Z))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add nozzle tool to scene!");
    return;
  }

  if (!scene.addBrushTool(BRUSH_X, BRUSH_Y, BRUSH_Z))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add brush tool to scene!");
    return;
  }

  RCLCPP_INFO(LOGGER, "All scene objects added successfully!");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Set the positions for use in MTC task
  nozzle_handle_pose_.position.x = NOZZLE_X;
  nozzle_handle_pose_.position.y = NOZZLE_Y;
  nozzle_handle_pose_.position.z = NOZZLE_Z + 0.025 + 0.05; // tip_height/2 + handle_length/2
  nozzle_handle_pose_.orientation.w = 1.0;

  brush_handle_pose_.position.x = BRUSH_X;
  brush_handle_pose_.position.y = BRUSH_Y;
  brush_handle_pose_.position.z = BRUSH_Z + (BRUSH_HEAD_HEIGHT / 2.0) + (HANDLE_LENGTH / 2.0); // tip_height/2 + handle_length/2
  brush_handle_pose_.orientation.w = 1.0;

  mirror_pose_.position.x = MIRROR_X;
  mirror_pose_.position.y = MIRROR_Y;
  mirror_pose_.position.z = MIRROR_Z;
  mirror_pose_.orientation.w = 1.0;

  // Set mirror dimensions (from ToiletScene)
  mirror_width_ = 0.6;
  mirror_height_ = 0.4;

  ///////////////// test pose..works 

  //   moveit_msgs::msg::CollisionObject object;
  // moveit::planning_interface::PlanningSceneInterface psi;

  // object.id = "brush_handle";
  // object.header.frame_id = "world";
  // object.primitives.resize(1);
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // object.primitives[0].dimensions = {0.1, 0.02}; // height, radius

  // // geometry_msgs::msg::Pose pose;
  // // pose.position.x = 0.35;
  // // pose.position.y = 0.2;
  // // pose.position.z = 0.10; // Half the height above ground
  // // pose.orientation.w = 1.0;
  // // test pose
  // geometry_msgs::msg::Pose pose;
  // pose.position.x = 0.5;
  // pose.position.y = -0.25;
  // pose.position.z = 0.1; // Half the height above ground
  // pose.orientation.w = 1.0;
  // // Fix: Use the correct method for setting pose
  // object.primitive_poses.resize(1);
  // object.primitive_poses[0] = pose;
  // object.operation = object.ADD;

  // //
  // psi.addCollisionObjects({object});

  // // Wait for scene and MoveIt to be ready
  // rclcpp::sleep_for(std::chrono::seconds(2));

  // // Fix: Set nozzle handle pose correctly
  //  brush_handle_pose_ = pose;

  // RCLCPP_INFO(node_->get_logger(), "Added test object at x=%.3f, y=%.3f, z=%.3f",
  //  pose.position.x, pose.position.y, pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "brush handle at x=%.3f, y=%.3f, z=%.3f",
              brush_handle_pose_.position.x, brush_handle_pose_.position.y, brush_handle_pose_.position.z);
  RCLCPP_INFO(node_->get_logger(), "Nozzle handle at x=%.3f, y=%.3f, z=%.3f",
              nozzle_handle_pose_.position.x, nozzle_handle_pose_.position.y, nozzle_handle_pose_.position.z);

  RCLCPP_INFO(LOGGER, "Scene setup complete!");
}

void MTCSprayTaskNode::planAllTasks()
{
  RCLCPP_INFO(LOGGER, " Planning All Tasks ");

  // Plan nozzle task
  nozzle_task_ = createNozzleTask();
  try
  {
    nozzle_task_.init();
    if (!nozzle_task_.plan(10))
    {
      RCLCPP_ERROR(LOGGER, "Nozzle task planning failed");
      return;
    }
    RCLCPP_INFO(LOGGER, "Nozzle task planned successfully");
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Nozzle task init failed: " << e);
    return;
  }

  // Plan brush task
  brush_task_ = createBrushTask();
  try
  {
    brush_task_.init();
    if (!brush_task_.plan(10))
    {
      RCLCPP_ERROR(LOGGER, "Brush task planning failed");
      return;
    }
    RCLCPP_INFO(LOGGER, " Brush task planned successfully");
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Brush task init failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, " All tasks planned! Ready for execution.");
}

void MTCSprayTaskNode::executeAllTasks()
{
  auto const draw_title = [this](auto text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0; // Place text 1m above the base link
      return msg;
    }();
    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE,
                               rviz_visual_tools::XLARGE);
    visual_tools_->trigger();
  };

  draw_title("Starting Spray Task");
  visual_tools_->trigger();
  // Execute nozzle task
  auto nozzle_result = nozzle_task_.execute(*nozzle_task_.solutions().front());
  if (nozzle_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Nozzle execution failed");
    return;
  }

  // Small pause between tasks
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Execute brush task
  draw_title("Starting Brush Swiping Task");
  auto brush_result = brush_task_.execute(*brush_task_.solutions().front());
  if (brush_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Brush execution failed");
    return;
  }

}

mtc::Task MTCSprayTaskNode::createNozzleTask()
{
  mtc::Task task;
  task.stages()->setName("Nozzle Spray Task");
  task.loadRobotModel(node_);

  mtc::Stage *current_state_ptr = nullptr;

  const auto &arm_group = "panda_arm";
  const auto &hand_group = "hand";
  const auto &hand_frame = "panda_hand";

  task.setProperty("group", arm_group);
  task.setProperty("eef", hand_group);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.005);

  //  Current State
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  // Open Hand
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  open_hand->setGroup(hand_group);
  open_hand->setGoal("open");
  task.add(std::move(open_hand));

  // Move to pick area
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick nozzle",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage *attach_object_stage = nullptr;

  // NOZZLE PICKUP SEQUENCE.. this can function for same pattern for other tools
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick nozzle");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach nozzle", cartesian_planner);
      stage->properties().set("marker_ns", "approach_nozzle");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Generate grasp pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate nozzle grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "nozzle_grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("nozzle_handle");
      stage->setAngleDelta(M_PI / 4);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.15;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("nozzle grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // Allow collisions for grasping
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, nozzle)");
      stage->allowCollisions("nozzle_handle",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), true);
      stage->allowCollisions("nozzle_tip",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), true);
      grasp->insert(std::move(stage));
    }

    // Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    // Attach objects
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach nozzle");
      stage->attachObject("nozzle_handle", hand_frame);
      stage->attachObject("nozzle_tip", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift nozzle
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift nozzle", cartesian_planner);
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_nozzle");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // SPRAY SEQUENCE.. can pick any order
  auto corner_poses = calculateMirrorCorners();
  std::vector<size_t> spray_order = {2, 1, 0, 3}; // clockwise from bottom-left
  std::vector<std::string> corner_names = {"bottom-right", "top-right", "top-left", "bottom-left"};

  for (size_t i = 0; i < spray_order.size(); ++i)
  {
    size_t corner_idx = spray_order[i];

    auto spray_corner = std::make_unique<mtc::stages::MoveTo>(
        "spray " + corner_names[i], sampling_planner);
    spray_corner->setGroup(arm_group);
    spray_corner->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    spray_corner->setTimeout(2); 

    geometry_msgs::msg::PoseStamped corner_target;
    corner_target.header.frame_id = "world";
    corner_target.pose = corner_poses[corner_idx];

    corner_target.pose.orientation.x = 0.0;
    corner_target.pose.orientation.y = 0.707;
    corner_target.pose.orientation.z = 0.0;
    corner_target.pose.orientation.w = 0.707;

    spray_corner->setGoal(corner_target);
    task.add(std::move(spray_corner));
  }

  // PLACE NOZZLE BACK
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place nozzle",
        mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));

    auto place_tool = std::make_unique<mtc::SerialContainer>("place nozzle");
    task.properties().exposeTo(place_tool->properties(), {"eef", "group", "ik_frame"});
    place_tool->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach placement
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach place position", cartesian_planner);
      stage->properties().set("marker_ns", "approach_place");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place_tool->insert(std::move(stage));
    }

    // Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate nozzle place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "nozzle_place_pose");
      stage->setObject("nozzle_handle");

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "world";
      place_pose.pose = nozzle_handle_pose_;
      stage->setPose(place_pose);
      stage->setMonitoredStage(attach_object_stage);

      Eigen::Isometry3d place_frame_transform;
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      place_frame_transform.linear() = q.matrix();
      place_frame_transform.translation().z() = 0.15;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("nozzle place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place_tool->insert(std::move(wrapper));
    }

    // Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place_tool->insert(std::move(stage));
    }

    // Detach objects
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach nozzle");
      stage->detachObject("nozzle_handle", hand_frame);
      stage->detachObject("nozzle_tip", hand_frame);
      place_tool->insert(std::move(stage));
    }

    // Forbid collisions
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand, nozzle)");
      stage->allowCollisions("nozzle_handle",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), false);
      stage->allowCollisions("nozzle_tip",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), false);
      place_tool->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from nozzle", cartesian_planner);
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      place_tool->insert(std::move(stage));
    }

    task.add(std::move(place_tool));
  }

  // Return home
  {
    auto return_home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    return_home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    return_home->setGoal("ready");
    task.add(std::move(return_home));
  }

  return task;
}

mtc::Task MTCSprayTaskNode::createBrushTask()
{
  mtc::Task task;
  task.stages()->setName("Brush Task");
  task.loadRobotModel(node_);

  mtc::Stage *current_state_ptr = nullptr;

  const auto &arm_group = "panda_arm";
  const auto &hand_group = "hand";
  const auto &hand_frame = "panda_hand";

  task.setProperty("group", arm_group);
  task.setProperty("eef", hand_group);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.005);

  // Current State
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  // Open Hand
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  open_hand->setGroup(hand_group);
  open_hand->setGoal("open");
  task.add(std::move(open_hand));

  // Move to brush
  auto stage_move_to_pick_brush = std::make_unique<mtc::stages::Connect>(
      "move to pick brush",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
  stage_move_to_pick_brush->setTimeout(5.0);
  stage_move_to_pick_brush->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick_brush));

  mtc::Stage *attach_brush_stage = nullptr;

  // BRUSH PICKUP SEQUENCE
  {
    auto grasp_brush = std::make_unique<mtc::SerialContainer>("pick brush");
    task.properties().exposeTo(grasp_brush->properties(), {"eef", "group", "ik_frame"});
    grasp_brush->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach brush", cartesian_planner);
      stage->properties().set("marker_ns", "approach_brush");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp_brush->insert(std::move(stage));
    }

    // Generate grasp pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate brush grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "brush_grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("brush_handle");
      stage->setAngleDelta(M_PI / 4);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_brush_frame_transform;
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      grasp_brush_frame_transform.linear() = q.matrix();
      grasp_brush_frame_transform.translation().z() = 0.15;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("brush grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_brush_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp_brush->insert(std::move(wrapper));
    }

    // Allow collisions with handle and head
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, brush)");
      stage->allowCollisions("brush_handle",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), true);
      stage->allowCollisions("brush_head",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), true);
      grasp_brush->insert(std::move(stage));
    }

    // Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("close");
      grasp_brush->insert(std::move(stage));
    }

    // Attach objects
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach brush");
      stage->attachObject("brush_handle", hand_frame);
      stage->attachObject("brush_head", hand_frame);
      attach_brush_stage = stage.get();
      grasp_brush->insert(std::move(stage));
    }

    // Lift brush
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift brush", cartesian_planner);
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_brush");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      grasp_brush->insert(std::move(stage));
    }

    task.add(std::move(grasp_brush));
  }

  // Move to cleaning start position
  auto move_to_clean = std::make_unique<mtc::stages::Connect>(
      "move to cleaning area",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
  move_to_clean->setTimeout(5.0);
  move_to_clean->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_to_clean));

  {
    auto clean_path = std::make_unique<mtc::SerialContainer>("clean mirror");
    clean_path->setProperty("group", arm_group);

    // Constants from declared glbals
    double mirror_x = MIRROR_X;
    double mirror_y_start = MIRROR_Y - SWIPE_DISTANCE / 2.0;
    double mirror_y_end = MIRROR_Y + SWIPE_DISTANCE / 2.0;

    double mirror_z_top = MIRROR_Z + SWIPE_DISTANCE / 2.0;
    double mirror_z_bottom = MIRROR_Z - SWIPE_DISTANCE / 2.0;

    double approach_offset_x = -SPRAY_DISTANCE; // 0.4m in front of mirror
    double swipe_step = 0.1;                    // vertical spacing

    geometry_msgs::msg::Quaternion brush_orientation;
    brush_orientation.x = 0.0;
    brush_orientation.y = 0.707; // 90° rotation around Y axis
    brush_orientation.z = 0.0;
    brush_orientation.w = 0.707;

    bool left_to_right = true;

    for (double z = mirror_z_top; z >= mirror_z_bottom; z -= swipe_step)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = mirror_x + approach_offset_x;
      pose.pose.position.y = left_to_right ? mirror_y_start : mirror_y_end;
      pose.pose.position.z = z;
      pose.pose.orientation = brush_orientation;

      auto move_to_start = std::make_unique<mtc::stages::MoveTo>("move to stroke", sampling_planner);
      move_to_start->setGroup(arm_group);
      move_to_start->setGoal(pose);
      clean_path->insert(std::move(move_to_start));

      // Swipe stroke along Y axis
      auto swipe = std::make_unique<mtc::stages::MoveRelative>("swipe stroke", cartesian_planner);
      swipe->properties().set("marker_ns", "swipe");
      swipe->setGroup(arm_group);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.x = 0.0;
      dir.vector.y = left_to_right ? (mirror_y_end - mirror_y_start) : -(mirror_y_end - mirror_y_start);

      dir.vector.z = 0.0;
      swipe->setDirection(dir);

      clean_path->insert(std::move(swipe));
      left_to_right = !left_to_right; // flip direction
    }

    auto retreat_after_cleaning = std::make_unique<mtc::stages::MoveTo>("retreat after cleaning", sampling_planner);
    retreat_after_cleaning->setGroup(arm_group);

    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.header.frame_id = "world";
    final_pose.pose.position.x = mirror_x + approach_offset_x; // Same distance from mirror
    final_pose.pose.position.y = MIRROR_Y;                     // Center of mirror
    final_pose.pose.position.z = MIRROR_Z + 0.2;               // Slightly above mirror center
    final_pose.pose.orientation = brush_orientation;           // Same orientation as cleaning

    retreat_after_cleaning->setGoal(final_pose);
    clean_path->insert(std::move(retreat_after_cleaning));

    task.add(std::move(clean_path));
  }

  {

    auto place_tool = std::make_unique<mtc::SerialContainer>("place brush");
    task.properties().exposeTo(place_tool->properties(), {"eef", "group", "ik_frame"});
    place_tool->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach placement
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach place position", cartesian_planner);
      stage->properties().set("marker_ns", "approach_place");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place_tool->insert(std::move(stage));
    }

    // Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate brush place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "brush_place_pose");
      stage->setObject("brush_handle");

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "world";
      place_pose.pose = brush_handle_pose_;
      stage->setPose(place_pose);
      stage->setMonitoredStage(attach_brush_stage);

      Eigen::Isometry3d place_frame_transform;
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      place_frame_transform.linear() = q.matrix();
      place_frame_transform.translation().z() = 0.15;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("brush place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place_tool->insert(std::move(wrapper));
    }

    // Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place_tool->insert(std::move(stage));
    }

    // Detach objects
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach brush");
      stage->detachObject("brush_handle", hand_frame);
      stage->detachObject("brush_head", hand_frame);
      place_tool->insert(std::move(stage));
    }

    // Forbid collisions
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand, brush)");
      stage->allowCollisions("brush_handle",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), false);
      stage->allowCollisions("brush_head",
                             task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(), false);
      place_tool->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from brush", cartesian_planner);
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      place_tool->insert(std::move(stage));
    }

    task.add(std::move(place_tool));
  }

  // Return home with brush
  {
    auto return_home = std::make_unique<mtc::stages::MoveTo>("return home with brush", interpolation_planner);
    return_home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    return_home->setGoal("ready");
    task.add(std::move(return_home));
  }

  return task;
}

// Update your main() function:
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCSprayTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
  executor.add_node(mtc_task_node->getNodeBaseInterface());
  executor.spin();
  executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  mtc_task_node->setupPlanningScene();

  // Plan everything first.. mmaybe split tasks further down into 4-5 modules
  mtc_task_node->planAllTasks();

  // Execute everything.. 
  mtc_task_node->executeAllTasks();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}