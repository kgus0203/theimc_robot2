#include "behaviortree_cpp_v3/bt_factory.h"
#include "theimc_bt_nodes/drive_cmd_vel.hpp"
#include "theimc_bt_nodes/for_each_rail.hpp"
#include "theimc_bt_nodes/get_rail_pose.hpp"
#include "theimc_bt_nodes/go_to_pose.hpp"
#include "theimc_bt_nodes/publish_rail_command.hpp"
#include "theimc_bt_nodes/rail_approach.hpp"
#include "theimc_bt_nodes/return_home_requested.hpp"
#include "theimc_bt_nodes/wait_for_rail_state.hpp"
#include "theimc_bt_nodes/wait_for_mission_trigger.hpp"
#include "theimc_bt_nodes/wait_seconds.hpp"
#include "theimc_bt_nodes/is_battery_low.hpp"
#include "theimc_bt_nodes/save_current_pose.hpp"
#include "theimc_bt_nodes/wait_for_charge.hpp"
#include "theimc_bt_nodes/toggle_docking.hpp"


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<theimc_bt_nodes::DriveCmdVel>("DriveCmdVel");
  factory.registerNodeType<theimc_bt_nodes::ForEachRail>("ForEachRail");
  factory.registerNodeType<theimc_bt_nodes::GetRailPose>("GetRailPose");
  factory.registerNodeType<theimc_bt_nodes::GoToPose>("GoToPose");
  factory.registerNodeType<theimc_bt_nodes::PublishRailCommand>("PublishRailCommand");
  factory.registerNodeType<theimc_bt_nodes::RailApproach>("RailApproach");
  factory.registerNodeType<theimc_bt_nodes::ReturnHomeRequested>("ReturnHomeRequested");
  factory.registerNodeType<theimc_bt_nodes::WaitForRailState>("WaitForRailState");
  factory.registerNodeType<theimc_bt_nodes::WaitForMissionTrigger>("WaitForMissionTrigger");
  factory.registerNodeType<theimc_bt_nodes::WaitSeconds>("WaitSeconds");
  factory.registerNodeType<theimc_bt_nodes::IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<theimc_bt_nodes::SaveCurrentPose>("SaveCurrentPose");
  factory.registerNodeType<theimc_bt_nodes::WaitForCharge>("WaitForCharge");
  factory.registerNodeType<theimc_bt_nodes::ToggleDocking>("ToggleDocking");
}
