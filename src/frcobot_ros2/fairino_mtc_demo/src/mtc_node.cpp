#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <thread> 


static const rclcpp::Logger LOGGER = rclcpp::get_logger("fairino_mtc_demo");

namespace mtc = moveit::task_constructor;

class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  ~MTCTaskNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();
    
  void setupPlanningScene();
  void doTask();
  void doReverseTask();

private:
  mtc::Task _createTask();
  mtc::Task _createReverseTask();
  mtc::Task _task;
  mtc::Task _reverse_task;
  rclcpp::Node::SharedPtr _node;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : _node{std::make_shared<rclcpp::Node>("fairino_mtc_demo_node", options)}
{}

MTCTaskNode::~MTCTaskNode() {}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::get_node_base_interface() {
  return _node->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene() {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.02};
  
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.2;
  pose.position.y = -0.35;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask() {
  _task = _createTask();
  RCLCPP_INFO(LOGGER, "start to init task!");
  try {
    _task.init();
  }
  catch(mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  try {
    if(!_task.plan(10)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed!");
    }
  } catch(moveit::task_constructor::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
  }

  for(auto item : _task.solutions()) {
    _task.introspection().publishSolution(*item);
    auto result = _task.execute(*item);
    if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed!");
      return;
    }
  }
  RCLCPP_INFO(LOGGER, "Task execution SUCCESS!");
}

mtc::Task MTCTaskNode::_createTask() {
  mtc::Task task;
  task.stages()->setName("fairino_mtc_task");
  task.loadRobotModel(_node);
  
  const auto& arm_group_name = "arm";
  const auto& gripper_group_name = "gripper"; 
  const auto& tip_frame = "gripper_grasp_link";
  
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", tip_frame);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));
  
  // Planners
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(_node);
  auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  std::string gripper_joint_name = "gripper_left_outer_knuckle_joint"; 

  std::map<std::string, double> open_pose;
  open_pose[gripper_joint_name] = 0.0; 

  std::map<std::string, double> close_pose;
  close_pose[gripper_joint_name] = 0.4; 

  // 0. Ensure gripper is open at the start
  auto stage_open_initial = std::make_unique<mtc::stages::MoveTo>("open gripper initial", joint_planner);
  stage_open_initial->setGroup(gripper_group_name);
  stage_open_initial->setGoal(open_pose);
  task.add(std::move(stage_open_initial));

  // 1. Move to a point hovering right above the object using a free-space planner
  auto stage_move_robot = std::make_unique<mtc::stages::MoveTo>("movepos1", sampling_planner);
  stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot->setIKFrame(tip_frame);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "world";
  target_pose.pose.position.x = 0.2;
  target_pose.pose.position.y = -0.35; 
  target_pose.pose.position.z = 0.25;  
  target_pose.pose.orientation.w = 1.0; 

  stage_move_robot->setGoal(target_pose);
  task.add(std::move(stage_move_robot));

  // 2. Approach object
  auto stage = std::make_unique<mtc::stages::MoveRelative>("approch object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.03, 0.2); 
  stage->setIKFrame(tip_frame);
  stage->properties().set("marker_ns", "approch object");
  
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage->setDirection(vec);
  task.add(std::move(stage));

  // 2.3 ALLOW COLLISION: Tell MoveIt it's okay for the gripper links to touch the object
  auto stage_allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper object collision");
  stage_allow_collision->allowCollisions("object", 
    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNames(), true);
  task.add(std::move(stage_allow_collision));

  // 2.5 Actuate the fingers to close around the physical object
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
  stage_close->setGroup(gripper_group_name);
  stage_close->setGoal(close_pose);
  task.add(std::move(stage_close));

  // 3. Attach object (updates planning scene collision matrix fundamentally)
  auto stage_grap = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage_grap->attachObject("object", tip_frame);
  task.add(std::move(stage_grap));

  // 4. Lift object
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("liftobj", cartesian_planner);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.03, 0.2);
  stage_lift->setIKFrame(tip_frame);
  stage_lift->properties().set("marker_ns", "liftobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;
  stage_lift->setDirection(vec);
  task.add(std::move(stage_lift));

  // 5. Move to position 2
  auto stage_move_robot2 = std::make_unique<mtc::stages::MoveRelative>("movepos2", cartesian_planner);
  stage_move_robot2->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot2->setMinMaxDistance(0.03, 0.2);
  stage_move_robot2->setIKFrame(tip_frame);
  stage_move_robot2->properties().set("marker_ns", "movepos2");
  vec.vector.x = -1;
  vec.vector.y = 0;
  vec.vector.z = 0;
  stage_move_robot2->setDirection(vec);
  task.add(std::move(stage_move_robot2));

  // 6. Lower object
  auto stage_put = std::make_unique<mtc::stages::MoveRelative>("putobj", cartesian_planner);
  stage_put->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_put->setMinMaxDistance(0.03, 0.2);
  stage_put->setIKFrame(tip_frame);
  stage_put->properties().set("marker_ns", "putobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage_put->setDirection(vec);
  task.add(std::move(stage_put));

  // 7. Detach object
  auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage_detach->detachObject("object", tip_frame);
  task.add(std::move(stage_detach));

  // 7.5 Actuate fingers to reopen back up
  auto stage_open_final = std::make_unique<mtc::stages::MoveTo>("open gripper final", joint_planner);
  stage_open_final->setGroup(gripper_group_name);
  stage_open_final->setGoal(open_pose);
  task.add(std::move(stage_open_final));

  // 8. Return home
  auto stage_end = std::make_unique<mtc::stages::MoveRelative>("returnhome", cartesian_planner);
  stage_end->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_end->setMinMaxDistance(0.03, 0.2);
  stage_end->setIKFrame(tip_frame);
  stage_end->properties().set("marker_ns", "returnhome");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;
  stage_end->setDirection(vec);
  task.add(std::move(stage_end));

  return task;
}
void MTCTaskNode::doReverseTask() {
  _task.clear();
  _task = _createReverseTask();
  RCLCPP_INFO(LOGGER, "start to init reverse task!");
  try {
    _task.init();
  }
  catch(mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  try {
    if(!_task.plan(10)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Reverse Task planning failed!");
    }
  } catch(moveit::task_constructor::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
  }

  for(auto item : _task.solutions()) {
    _task.introspection().publishSolution(*item);
    auto result = _task.execute(*item);
    if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Reverse Task execution failed!");
      return;
    }
  }
  RCLCPP_INFO(LOGGER, "Reverse Task execution SUCCESS!");
  _task.clear();
}

mtc::Task MTCTaskNode::_createReverseTask() {
  mtc::Task task;
  task.stages()->setName("fairino_mtc_reverse_task");
  task.loadRobotModel(_node);
  
  const auto& arm_group_name = "arm";
  const auto& tip_frame = "wrist3_link";
  
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", tip_frame);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  // 1. Approach object
  auto stage = std::make_unique<mtc::stages::MoveRelative>("approch object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.2);
  stage->setIKFrame(tip_frame);
  stage->properties().set("marker_ns", "approch object");
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage->setDirection(vec);
  task.add(std::move(stage));
 
  // 2. Attach object
  auto stage_grap = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage_grap->attachObject("object", tip_frame);
  task.add(std::move(stage_grap));

  // 3. Lift object
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("liftobj", cartesian_planner);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.1, 0.2);
  stage_lift->setIKFrame(tip_frame);
  stage_lift->properties().set("marker_ns", "liftobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;
  stage_lift->setDirection(vec);
  task.add(std::move(stage_lift));

  // 4. Move back
  auto stage_moveback = std::make_unique<mtc::stages::MoveRelative>("moveback", cartesian_planner);
  stage_moveback->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_moveback->setMinMaxDistance(0.1, 0.2);
  stage_moveback->setIKFrame(tip_frame);
  stage_moveback->properties().set("marker_ns", "moveback");
  vec.vector.x = 1;
  vec.vector.y = 0;
  vec.vector.z = 0;
  stage_moveback->setDirection(vec);
  task.add(std::move(stage_moveback));

  // 5. Put object
  auto stage_put = std::make_unique<mtc::stages::MoveRelative>("putobj", cartesian_planner);
  stage_put->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_put->setMinMaxDistance(0.1, 0.2);
  stage_put->setIKFrame(tip_frame);
  stage_put->properties().set("marker_ns", "putobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage_put->setDirection(vec);
  task.add(std::move(stage_put));

  // 6. Detach object
  auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage_detach->detachObject("object", tip_frame);
  task.add(std::move(stage_detach));

  // 7. Lift Arm
  auto stage_liftarm = std::make_unique<mtc::stages::MoveRelative>("liftarm", cartesian_planner);
  stage_liftarm->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_liftarm->setMinMaxDistance(0.1, 0.2);
  stage_liftarm->setIKFrame(tip_frame);
  stage_liftarm->properties().set("marker_ns", "liftarm");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;
  stage_liftarm->setDirection(vec);
  task.add(std::move(stage_liftarm));

  // 8. Return to Start
  auto stage_returnstart = std::make_unique<mtc::stages::MoveRelative>("returnstart", cartesian_planner);
  stage_returnstart->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_returnstart->setMinMaxDistance(0.1, 0.2);
  stage_returnstart->setIKFrame(tip_frame);
  stage_returnstart->properties().set("marker_ns", "returnstart");
  vec.vector.x = 0;
  vec.vector.y = 1;
  vec.vector.z = 0;
  stage_returnstart->setDirection(vec);
  task.add(std::move(stage_returnstart));

  return task;
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  // Direct Code Fix: Force MoveIt to look for "gripper" instead of "gripper_controller"
  options.append_parameter_override("moveit_simple_controller_manager.controller_names", 
    std::vector<std::string>{"arm_controller", "gripper"});
  
  options.append_parameter_override("moveit_simple_controller_manager.gripper.type", 
    "FollowJointTrajectory");
  
  options.append_parameter_override("moveit_simple_controller_manager.gripper.action_ns", 
    "follow_joint_trajectory");
  
  options.append_parameter_override("moveit_simple_controller_manager.gripper.default", 
    true);
  
  options.append_parameter_override("moveit_simple_controller_manager.gripper.joints", 
    std::vector<std::string>{"gripper_left_outer_knuckle_joint"});
  
  // Pass our modified options into the task node constructor
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::thread([&executor, mtc_task_node]() {
    executor.add_node(mtc_task_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(mtc_task_node->get_node_base_interface());
  });
  
  // Now run your actual MTC logic on the main thread
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();
  
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  
  rclcpp::shutdown();
  return 0;
}