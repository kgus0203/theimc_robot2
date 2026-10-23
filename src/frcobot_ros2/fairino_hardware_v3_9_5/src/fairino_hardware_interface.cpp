#include "fairino_hardware/fairino_hardware_interface.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (sysinfo.joints.size() != 6) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Expected exactly six joints");
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::set<std::string> seen;
    for (size_t i = 0; i < sysinfo.joints.size(); ++i) {
        const auto& name = sysinfo.joints[i].name;
        if (name.size() != 2 || name[0] != 'j' || name[1] < '1' || name[1] > '6' ||
            !seen.insert(name).second) {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Expected unique joints j1..j6");
            return hardware_interface::CallbackReturn::ERROR;
        }
        _sdk_joint_index[i] = static_cast<size_t>(name[1] - '1');
    }
    const auto ip = sysinfo.hardware_parameters.find("robot_ip");
    if (ip != sysinfo.hardware_parameters.end()) _controller_ip = ip->second;
    info_ = sysinfo;//info_是父类中定义的变量
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
    using namespace std::chrono_literals;
    const auto logger = rclcpp::get_logger("FairinoHardwareInterface");
    _faulted = false;
    _command_active = false;
    _servo_started = false;
    _control_mode = 0;
    _timing_samples = 0;
    _servo_over_10ms = 0;
    _servo_over_20ms = 0;
    _servo_consecutive_over_10ms = 0;
    _servo_max_consecutive_over_10ms = 0;
    _max_read_us = 0;
    _max_servo_us = 0;
    _max_period_us = 0;
    _ptr_robot = std::make_unique<FRRobot>();
    auto fail = [&](const char* operation, int code) {
        RCLCPP_ERROR(logger, "%s failed (SDK code %d). Check robot enable/auto mode, alarms and connection; hardware activation aborted.", operation, code);
        disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    };
    int code = _ptr_robot->RPC(_controller_ip.c_str());
    if (code != 0) return fail("RPC", code);
    rclcpp::sleep_for(200ms);
    JointPos actual{};
    code = _ptr_robot->GetActualJointPosDegree(0, &actual);
    if (code != 0) return fail("GetActualJointPosDegree", code);
    for (size_t i = 0; i < 6; ++i) {
        const double value = actual.jPos[_sdk_joint_index[i]];
        if (!std::isfinite(value)) return fail("Non-finite initial joint position", -1);
        _jnt_position_state[i] = value * M_PI / 180.0;
        _jnt_position_command[i] = _jnt_position_state[i];
        _jnt_velocity_command[i] = _jnt_torque_command[i] = 0;
        _jnt_velocity_state[i] = _jnt_torque_state[i] = 0;
    }

    code = _ptr_robot->ResetAllError();
    if (code != 0) return fail("ResetAllError", code);
    rclcpp::sleep_for(1s);
    int main_code = 0;
    int sub_code = 0;
    code = _ptr_robot->GetRobotErrorCode(&main_code, &sub_code);
    RCLCPP_INFO(logger, "Robot error query: call=%d, main=%d, sub=%d",
                code, main_code, sub_code);
    if (code != 0) return fail("GetRobotErrorCode", code);
    if (main_code != 0 || sub_code != 0) {
        RCLCPP_ERROR(logger, "Robot alarm remains after ResetAllError: %d/%d",
                     main_code, sub_code);
        disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    code = _ptr_robot->RobotEnable(1);
    if (code != 0) return fail("RobotEnable(1)", code);
    ROBOT_STATE_PKG status{};
    bool enabled = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        rclcpp::sleep_for(100ms);
        status = ROBOT_STATE_PKG{};
        code = _ptr_robot->GetRobotRealTimeState(&status);
        if (code != 0) return fail("GetRobotRealTimeState after RobotEnable(1)", code);
        if (status.rbtEnableState == 1) {
            enabled = true;
            break;
        }
    }
    if (!enabled) return fail("RobotEnable(1) did not reach enabled state", -1);
    RCLCPP_INFO(logger, "Fresh robot status: mode=%u (0=auto), enabled=%d, state=%u",
                status.robot_mode, status.rbtEnableState, status.robot_state);
    if (status.robot_mode != 0) {
        RCLCPP_WARN(logger, "Robot is in manual mode; requesting automatic mode with Mode(0)");
        code = _ptr_robot->Mode(0);
        if (code != 0) return fail("Mode(0)", code);

        bool automatic = false;
        for (int attempt = 0; attempt < 10; ++attempt) {
            rclcpp::sleep_for(100ms);
            code = _ptr_robot->GetRobotRealTimeState(&status);
            if (code != 0) return fail("GetRobotRealTimeState after Mode(0)", code);
            if (status.robot_mode == 0) {
                automatic = true;
                break;
            }
        }
        if (!automatic) return fail("Mode(0) did not reach automatic mode", -1);
        RCLCPP_INFO(logger, "Robot mode changed to automatic");
    }
    // A controller left at 0% global speed rejects ServoJ with SDK code 172.
    // MoveIt still limits the actual motion through its time-parameterized
    // position targets and velocity/acceleration scaling.
    code = _ptr_robot->SetSpeed(100);
    if (code != 0) return fail("SetSpeed(100)", code);
    RCLCPP_INFO(logger, "Robot global automatic speed set to 100%%");
    code = _ptr_robot->StopMotion();
    if (code != 0) return fail("StopMotion before ServoMoveStart", code);
    code = _ptr_robot->ServoMoveEnd();
    if (code != 0) return fail("ServoMoveEnd before ServoMoveStart", code);
    code = _ptr_robot->ServoMoveStart();
    if (code != 0) return fail("ServoMoveStart", code);
    _servo_started = true;
    RCLCPP_INFO(logger, "Hardware connected at %s; ServoMoveStart succeeded",
                _controller_ip.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

void FairinoHardwareInterface::disconnect()
{
    if (!_ptr_robot) return;
    if (_servo_started) {
        _ptr_robot->ServoMoveEnd();
        _ptr_robot->StopMotion();
    }
    _servo_started = false;
    _command_active = false;
    _ptr_robot->CloseRPC();
    _ptr_robot.reset();
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_error(const rclcpp_lifecycle::State&)
{
    _faulted = true;
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FairinoHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>&)
{
    size_t relevant_starts = 0;
    for (const auto& interface : start_interfaces) {
        for (const auto& joint : info_.joints) {
            if (interface == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
                ++relevant_starts;
                break;
            }
        }
    }
    if (relevant_starts != 0 && relevant_starts != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "All six position interfaces must start together (received %zu)",
                     relevant_starts);
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    const auto is_ours = [this](const std::string& interface) {
        return std::any_of(info_.joints.begin(), info_.joints.end(),
            [&interface](const auto& joint) {
                return interface == joint.name + "/" + hardware_interface::HW_IF_POSITION;
            });
    };
    const bool stopping = std::any_of(stop_interfaces.begin(), stop_interfaces.end(), is_ours);
    const bool starting = std::any_of(start_interfaces.begin(), start_interfaces.end(), is_ours);

    if (stopping) {
        _command_active = false;
        if (_ptr_robot && _servo_started) {
            _ptr_robot->ServoMoveEnd();
            _ptr_robot->StopMotion();
            _servo_started = false;
        }
    }
    if (starting) {
        for (size_t i = 0; i < info_.joints.size(); ++i)
            _jnt_position_command[i] = _jnt_position_state[i];
        _command_active = true;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Position command interfaces active; ServoJ enabled");
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (!_ptr_robot || _faulted) return hardware_interface::return_type::ERROR;
    JointPos actual{};
    const auto read_begin = std::chrono::steady_clock::now();
    const int code = _ptr_robot->GetActualJointPosDegree(1, &actual);
    const auto read_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - read_begin).count();
    _max_read_us = std::max(_max_read_us, read_us);
    if (code != 0 || std::any_of(actual.jPos, actual.jPos + 6,
                               [](double value) { return !std::isfinite(value); })) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "Joint feedback failed (SDK code %d); stopping hardware", code);
        _faulted = true;
        _ptr_robot->StopMotion();
        return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < 6; ++i)
        _jnt_position_state[i] = actual.jPos[_sdk_joint_index[i]] * M_PI / 180.0;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
    if (!_ptr_robot || _faulted) return hardware_interface::return_type::ERROR;
    if (!_command_active) return hardware_interface::return_type::OK;
    if (std::any_of(_jnt_position_command, _jnt_position_command + 6,
                    [](double value) { return !std::isfinite(value); })) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Non-finite joint command; stopping hardware");
        _faulted = true;
        _ptr_robot->StopMotion();
        return hardware_interface::return_type::ERROR;
    }
    JointPos command{};
    ExaxisPos external{0, 0, 0, 0};
    for (size_t i = 0; i < 6; ++i)
        command.jPos[_sdk_joint_index[i]] = _jnt_position_command[i] * 180.0 / M_PI;
    const double seconds = period.seconds();
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 0.1) {
        _faulted = true;
        _ptr_robot->StopMotion();
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Invalid ServoJ control period: %.6f", seconds);
        return hardware_interface::return_type::ERROR;
    }
    const auto period_us = static_cast<int64_t>(seconds * 1000000.0);
    _max_period_us = std::max(_max_period_us, period_us);
    if (!_servo_started) {
        const int start_code = _ptr_robot->ServoMoveStart();
        if (start_code != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "ServoMoveStart failed (SDK code %d)", start_code);
            _faulted = true;
            return hardware_interface::return_type::ERROR;
        }
        _servo_started = true;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoMoveStart succeeded; sending first ServoJ immediately");
    }
    // The controller firmware rejects zero ServoJ speed with SDK code 172.
    // Keep cmdT below the 100 Hz host period so transport and scheduler
    // overhead cannot accumulate commands in the robot's servo queue.
    // MoveIt still determines the physical motion rate through successive
    // position targets; these values only keep the servo command valid.
    constexpr float servo_acceleration = 100.0F;
    constexpr float servo_velocity = 100.0F;
    const auto servo_begin = std::chrono::steady_clock::now();
    const int code = _ptr_robot->ServoJ(
        &command, &external, servo_acceleration, servo_velocity, 0.008F, 0, 0);
    const auto servo_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - servo_begin).count();
    _max_servo_us = std::max(_max_servo_us, servo_us);
    _servo_samples_us[_timing_samples] = servo_us;
    ++_timing_samples;
    if (servo_us > 10000) {
        ++_servo_over_10ms;
        ++_servo_consecutive_over_10ms;
        _servo_max_consecutive_over_10ms = std::max(
            _servo_max_consecutive_over_10ms, _servo_consecutive_over_10ms);
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoJ deadline miss: %.3f ms, rc=%d, controller period=%.3f ms",
                    servo_us / 1000.0, code, period_us / 1000.0);
    } else {
        _servo_consecutive_over_10ms = 0;
    }
    if (servo_us > 20000) ++_servo_over_20ms;
    if (_timing_samples >= 1000) {
        auto sorted_servo_us = _servo_samples_us;
        std::sort(sorted_servo_us.begin(), sorted_servo_us.end());
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Control timing (1000 cycles): max read=%.3f ms, max ServoJ=%.3f ms, "
                    "ServoJ p95=%.3f ms, p99=%.3f ms, max period=%.3f ms, "
                    "ServoJ >10ms=%lu, >20ms=%lu, max consecutive >10ms=%lu",
                    _max_read_us / 1000.0, _max_servo_us / 1000.0,
                    sorted_servo_us[949] / 1000.0, sorted_servo_us[989] / 1000.0,
                    _max_period_us / 1000.0, _servo_over_10ms, _servo_over_20ms,
                    _servo_max_consecutive_over_10ms);
        _timing_samples = 0;
        _servo_over_10ms = 0;
        _servo_over_20ms = 0;
        _servo_consecutive_over_10ms = 0;
        _servo_max_consecutive_over_10ms = 0;
        _max_read_us = 0;
        _max_servo_us = 0;
        _max_period_us = 0;
    }
    if (code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "ServoJ rejected after %.3f ms (SDK code %d); stopping hardware instead of reporting success",
                     servo_us / 1000.0, code);
        int main_code = 0;
        int sub_code = 0;
        const int error_code = _ptr_robot->GetRobotErrorCode(&main_code, &sub_code);
        if (error_code == 0)
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Robot error at rejection: main=%d sub=%d", main_code, sub_code);
        _faulted = true;
        _ptr_robot->StopMotion();
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

} // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
