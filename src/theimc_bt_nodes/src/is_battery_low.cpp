#include "theimc_bt_nodes/is_battery_low.hpp"

namespace theimc_bt_nodes {

IsBatteryLow::IsBatteryLow(
    const std::string& xml_tag_name, 
    const BT::NodeConfiguration& config)
: BT::ConditionNode(xml_tag_name, config),
    current_battery_(100.0), 
    is_battery_low_(false),
    has_received_battery_state_(false)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    subscription_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);

            // [수정됨] double 타입 선언 및 0~100 단위로 들어올 경우(else) 대응
            double pct = msg->percentage; 
            if (pct <= 1.0 && pct >= 0.0) {
                current_battery_ = pct * 100.0; 
            } else {
                current_battery_ = pct;
            }
            has_received_battery_state_ = true;
        });
}

BT::PortsList IsBatteryLow::providedPorts() {
    return {
        BT::InputPort<double>("threshold", 20.0, "Battery percentage threshold to consider low"),
        BT::InputPort<double>("recovery_threshold", 90.0, "Battery percentage threshold to consider recovered")
    };
}

BT::NodeStatus IsBatteryLow::tick() {
    double threshold = 20.0;
    double recovery_threshold = 90.0;
    getInput("threshold", threshold);
    getInput("recovery_threshold", recovery_threshold);

    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_received_battery_state_) {
        // ROS 2 주기에 따라 로그가 너무 자주 찍히지 않도록 Throttle을 쓰거나 
        // WARN 레벨로 유지해도 무방합니다.
        //RCLCPP_WARN(node_->get_logger(), "No battery state received yet.");
        return BT::NodeStatus::FAILURE;
    }

    if (is_battery_low_) {
        if (current_battery_ >= recovery_threshold) {
            RCLCPP_INFO(node_->get_logger(), "Battery has recovered: %.2f%%", current_battery_);
            is_battery_low_ = false;
            return BT::NodeStatus::FAILURE;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Battery is still low: %.2f%%", current_battery_);
            return BT::NodeStatus::SUCCESS;
        }
    }

    if (current_battery_ < threshold) {
        RCLCPP_WARN(node_->get_logger(), "Battery is low: %.2f%%", current_battery_);
        is_battery_low_ = true;
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
} 

}  // namespace theimc_bt_nodes