#include "theimc_bt_nodes/wait_for_charge.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/battery_state.hpp>

namespace theimc_bt_nodes {

WaitForCharge::WaitForCharge(
    const std::string& xml_tag_name, 
    const BT::NodeConfiguration& config)
: BT::StatefulActionNode(xml_tag_name, config), current_battery_(0.0), has_received_data_(false) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    // 배터리 상태 토픽 구독
    subscription_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            double pct = msg->percentage;
            if (pct <= 1.0 && pct >= 0.0) {
                current_battery_ = pct * 100.0;
            } else {
                current_battery_ = pct;
            }
            has_received_data_ = true;
        });
}

BT::PortsList WaitForCharge::providedPorts() {
    return {
        BT::InputPort<double>("target_battery", 90.0, "Battery percentage to reach before success")
    };
}

BT::NodeStatus WaitForCharge::onStart() {
    if (!getInput("target_battery", target_battery_)) {
        RCLCPP_WARN(node_->get_logger(), "WaitForCharge missing target_battery input, using default %.1f", target_battery_);
    }

    // ★ 핵심 수정: 충전 대기 진입 시 이전 배터리 수신 기록을 리셋합니다.
    // 진입 '이후'에 들어오는 새로운 배터리 토픽으로만 충전 완료 여부를 판단합니다.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        has_received_data_ = false;
    }

    RCLCPP_INFO(node_->get_logger(), "WaitForCharge started. Waiting until battery reaches: %.2f%%", target_battery_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForCharge::onRunning() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 1. 충전 노드가 시작된 이후 아직 배터리 토픽을 수신하지 못했다면 RUNNING 대기
    if (!has_received_data_) {
        return BT::NodeStatus::RUNNING;
    }

    // 2. 수신된 배터리가 목표 수치(90%) 이상이면 SUCCESS 반환 후 다음(복귀 주행)으로 진행
    if (current_battery_ >= target_battery_) {
        RCLCPP_INFO(node_->get_logger(), "WaitForCharge complete. Current battery: %.2f%% >= Target: %.2f%%", 
                    current_battery_, target_battery_);
        return BT::NodeStatus::SUCCESS;
    }

    // 3. 목표치 미달 시 5초마다 로그를 출력하며 RUNNING 유지 (대기)
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
                        "Charging in progress... Current battery: %.2f%% / Target: %.2f%%", 
                        current_battery_, target_battery_);

    return BT::NodeStatus::RUNNING;
}

void WaitForCharge::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "WaitForCharge halted before reaching target battery.");
}

}  // namespace theimc_bt_nodes