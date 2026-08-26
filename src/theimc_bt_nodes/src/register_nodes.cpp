#include "behaviortree_cpp_v3/bt_factory.h"

#include "theimc_bt_nodes/clear_resume_target.hpp"
#include "theimc_bt_nodes/drive_cmd_vel.hpp"
#include "theimc_bt_nodes/drive_until_rail_state.hpp"
#include "theimc_bt_nodes/for_each_rail.hpp"
#include "theimc_bt_nodes/get_location_pose.hpp"
#include "theimc_bt_nodes/get_rail_pose.hpp"
#include "theimc_bt_nodes/go_to_pose.hpp"
#include "theimc_bt_nodes/is_battery_low.hpp"
#include "theimc_bt_nodes/load_resume_target.hpp"
#include "theimc_bt_nodes/publish_rail_command.hpp"
#include "theimc_bt_nodes/rail_approach.hpp"
#include "theimc_bt_nodes/rail_enter.hpp"
#include "theimc_bt_nodes/rail_exit.hpp"
#include "theimc_bt_nodes/rail_move_to_distance.hpp"
#include "theimc_bt_nodes/return_home_requested.hpp"
#include "theimc_bt_nodes/save_rail_progress.hpp"
#include "theimc_bt_nodes/toggle_docking.hpp"
#include "theimc_bt_nodes/wait_for_charge.hpp"
#include "theimc_bt_nodes/wait_for_mission_trigger.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<theimc_bt_nodes::WaitForMissionTrigger>(
    "WaitForMissionTrigger");

  factory.registerNodeType<theimc_bt_nodes::ReturnHomeRequested>(
    "ReturnHomeRequested");

  factory.registerNodeType<theimc_bt_nodes::ForEachRail>(
    "ForEachRail");

  factory.registerNodeType<theimc_bt_nodes::GetLocationPose>(
    "GetLocationPose");

  factory.registerNodeType<theimc_bt_nodes::GetRailPose>(
    "GetRailPose");

  factory.registerNodeType<theimc_bt_nodes::GoToPose>(
    "GoToPose");

  factory.registerNodeType<theimc_bt_nodes::DriveCmdVel>(
    "DriveCmdVel");

  factory.registerNodeType<theimc_bt_nodes::DriveUntilRailState>(
    "DriveUntilRailState");

  factory.registerNodeType<theimc_bt_nodes::RailApproach>(
    "RailApproach");

  factory.registerNodeType<theimc_bt_nodes::RailEnter>(
    "RailEnter");

  factory.registerNodeType<theimc_bt_nodes::RailExit>(
    "RailExit");

  factory.registerNodeType<theimc_bt_nodes::PublishRailCommand>(
    "PublishRailCommand");

  factory.registerNodeType<theimc_bt_nodes::RailMoveToDistance>(
    "RailMoveToDistance");

  factory.registerNodeType<theimc_bt_nodes::SaveRailProgress>(
    "SaveRailProgress");

  factory.registerNodeType<theimc_bt_nodes::LoadResumeTarget>(
    "LoadResumeTarget");

  factory.registerNodeType<theimc_bt_nodes::ClearResumeTarget>(
    "ClearResumeTarget");

  factory.registerNodeType<theimc_bt_nodes::IsBatteryLow>(
    "IsBatteryLow");

  factory.registerNodeType<theimc_bt_nodes::WaitForCharge>(
    "WaitForCharge");

  factory.registerNodeType<theimc_bt_nodes::ToggleDocking>(
    "ToggleDocking");
}
