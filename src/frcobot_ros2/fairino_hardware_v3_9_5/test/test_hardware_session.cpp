#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "fairino_hardware/fairino_hardware_interface.hpp"

using Driver = fairino_hardware::FairinoHardwareInterface;
using Callback = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;

class Session : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  Driver driver;
  void SetUp() override {
    FRRobot::read_code = FRRobot::start_code = FRRobot::write_code =
      FRRobot::mode_code = FRRobot::speed_code = 0;
    FRRobot::reset_error_code = FRRobot::error_query_code = FRRobot::enable_code = 0;
    FRRobot::starts = FRRobot::stops = FRRobot::ends = FRRobot::writes =
      FRRobot::closes = FRRobot::mode_calls = 0;
    FRRobot::speed_calls = FRRobot::last_speed = 0;
    FRRobot::reset_error_calls = 0;
    FRRobot::error_query_calls = FRRobot::enable_calls = 0;
    FRRobot::error_main = FRRobot::error_sub = 0;
    FRRobot::last_acceleration = FRRobot::last_velocity = 0;
    FRRobot::status = ROBOT_STATE_PKG{};
    for (int i = 0; i < 6; ++i) FRRobot::actual[i] = (i + 1) * 10;
    hardware_interface::HardwareInfo info;
    // Deliberately match the non-sorted joint-state order supplied by the user.
    for (const auto& name : {"j1", "j2", "j4", "j5", "j3", "j6"}) {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      hardware_interface::InterfaceInfo position;
      position.name = "position";
      joint.command_interfaces.push_back(position);
      joint.state_interfaces.push_back(position);
      info.joints.push_back(joint);
    }
    ASSERT_EQ(driver.on_init(info), Callback::SUCCESS);
  }
  void TearDown() override { driver.on_deactivate(rclcpp_lifecycle::State()); }
  Return write() { return driver.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)); }
  Return read() { return driver.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)); }
  Return startController() {
    return driver.perform_command_mode_switch(
      {"j1/position", "j2/position", "j3/position",
       "j4/position", "j5/position", "j6/position"}, {});
  }
};

TEST_F(Session, StartsServoAndSynchronizesMeasuredPositionsByName) {
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_EQ(FRRobot::starts, 1);
  EXPECT_EQ(FRRobot::reset_error_calls, 1);
  EXPECT_EQ(FRRobot::error_query_calls, 1);
  EXPECT_EQ(FRRobot::enable_calls, 1);
  EXPECT_EQ(FRRobot::speed_calls, 1);
  EXPECT_EQ(FRRobot::last_speed, 100);
  auto state = driver.export_state_interfaces();
  EXPECT_NEAR(state[2].get_value(), 40 * M_PI / 180, 1e-12);
  EXPECT_NEAR(state[4].get_value(), 30 * M_PI / 180, 1e-12);
  ASSERT_EQ(write(), Return::OK);
  EXPECT_EQ(FRRobot::starts, 1);
  EXPECT_EQ(FRRobot::writes, 0);
  ASSERT_EQ(startController(), Return::OK);
  ASSERT_EQ(write(), Return::OK);
  EXPECT_EQ(FRRobot::starts, 1);
  EXPECT_GT(FRRobot::last_acceleration, 0);
  EXPECT_GT(FRRobot::last_velocity, 0);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(FRRobot::last.jPos[i], (i + 1) * 10, 1e-10);
  driver.on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(FRRobot::ends, 2);
  EXPECT_EQ(FRRobot::closes, 1);
}
TEST_F(Session, RejectsFailedServoStart) {
  FRRobot::start_code = 14;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::writes, 0);
  EXPECT_EQ(FRRobot::closes, 1);
}
TEST_F(Session, PropagatesServoJErrorAndDoesNotRetryMotion) {
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(startController(), Return::OK);
  FRRobot::write_code = 14;
  EXPECT_EQ(write(), Return::ERROR);
  FRRobot::write_code = 0;
  EXPECT_EQ(write(), Return::ERROR);
  EXPECT_EQ(FRRobot::writes, 1);
  EXPECT_GT(FRRobot::stops, 0);
  driver.on_error(rclcpp_lifecycle::State());
  EXPECT_EQ(FRRobot::ends, 2);
}
TEST_F(Session, PropagatesFeedbackFailure) {
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  FRRobot::read_code = -1;
  EXPECT_EQ(read(), Return::ERROR);
  EXPECT_EQ(write(), Return::ERROR);
  EXPECT_EQ(FRRobot::writes, 0);
}
TEST_F(Session, ChecksSixthJointForNaN) {
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(startController(), Return::OK);
  auto commands = driver.export_command_interfaces();
  commands[5].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(write(), Return::ERROR);
  EXPECT_EQ(FRRobot::writes, 0);
}
TEST_F(Session, RejectsInvalidInitialFeedback) {
  FRRobot::actual[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::starts, 0);
}
TEST_F(Session, ChangesManualModeToAutomaticBeforeActivation) {
  FRRobot::status.robot_mode = 1;
  ASSERT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_EQ(FRRobot::mode_calls, 1);
  EXPECT_EQ(FRRobot::status.robot_mode, 0);
}
TEST_F(Session, RejectsFailedAutomaticModeChange) {
  FRRobot::status.robot_mode = 1;
  FRRobot::mode_code = 14;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::mode_calls, 1);
  EXPECT_EQ(FRRobot::closes, 1);
}
TEST_F(Session, RejectsFailedGlobalSpeedSetup) {
  FRRobot::speed_code = 172;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::speed_calls, 1);
  EXPECT_EQ(FRRobot::closes, 1);
}
TEST_F(Session, RejectsFailedErrorReset) {
  FRRobot::reset_error_code = 14;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::reset_error_calls, 1);
  EXPECT_EQ(FRRobot::speed_calls, 0);
  EXPECT_EQ(FRRobot::closes, 1);
}
TEST_F(Session, IgnoresStaleRealtimeAlarmAndUsesDirectErrorQuery) {
  FRRobot::status.main_code = 1;
  FRRobot::status.sub_code = 23;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_EQ(FRRobot::error_query_calls, 1);
}
TEST_F(Session, RejectsAlarmReportedByDirectErrorQuery) {
  FRRobot::error_main = 1;
  FRRobot::error_sub = 23;
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  EXPECT_EQ(FRRobot::starts, 0);
  EXPECT_EQ(FRRobot::closes, 1);
}
