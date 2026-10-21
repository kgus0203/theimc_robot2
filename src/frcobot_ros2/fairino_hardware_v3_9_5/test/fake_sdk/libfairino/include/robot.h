#pragma once
#include <algorithm>
struct JointPos { double jPos[6]; };
struct ExaxisPos { double ePos[4]; };
struct ROBOT_STATE_PKG { unsigned char robot_mode=0, robot_state=1; int rbtEnableState=1, main_code=0, sub_code=0; };
class FRRobot {
public:
  inline static int read_code = 0, start_code = 0, write_code = 0, mode_code = 0, speed_code = 0;
  inline static int reset_error_code = 0, error_query_code = 0, enable_code = 0;
  inline static int starts = 0, stops = 0, ends = 0, writes = 0, closes = 0, mode_calls = 0;
  inline static int speed_calls = 0, last_speed = 0, reset_error_calls = 0;
  inline static int error_query_calls = 0, enable_calls = 0, error_main = 0, error_sub = 0;
  inline static float last_acceleration = 0, last_velocity = 0;
  inline static ROBOT_STATE_PKG status{};
  inline static double actual[6] = {10, 20, 30, 40, 50, 60};
  inline static JointPos last{};
  int GetRobotRealTimeState(ROBOT_STATE_PKG* p) { *p = status; return 0; }
  int Mode(int mode) {
    ++mode_calls;
    if (mode_code == 0) status.robot_mode = static_cast<unsigned char>(mode);
    return mode_code;
  }
  int SetSpeed(int speed) {
    ++speed_calls; last_speed = speed; return speed_code;
  }
  int ResetAllError() {
    ++reset_error_calls;
    return reset_error_code;
  }
  int GetRobotErrorCode(int* main_code, int* sub_code) {
    ++error_query_calls; *main_code = error_main; *sub_code = error_sub;
    return error_query_code;
  }
  int RobotEnable(unsigned char enabled) {
    ++enable_calls;
    if (enable_code == 0) status.rbtEnableState = enabled;
    return enable_code;
  }
  int RPC(const char*) { return 0; }
  int GetActualJointPosDegree(int, JointPos* p) {
    std::copy(actual, actual + 6, p->jPos); return read_code;
  }
  int ServoMoveStart() { ++starts; return start_code; }
  int ServoMoveEnd() { ++ends; return 0; }
  int StopMotion() { ++stops; return 0; }
  void CloseRPC() { ++closes; }
  int ServoJ(JointPos* p, ExaxisPos*, float acc, float vel, float, float, float) {
    ++writes; last = *p; last_acceleration = acc; last_velocity = vel; return write_code;
  }
};
