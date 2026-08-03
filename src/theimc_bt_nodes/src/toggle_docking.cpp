#include "theimc_bt_nodes/toggle_docking.hpp"

namespace theimc_bt_nodes {

ToggleDocking::ToggleDocking(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {
    
    // 블랙보드에서 ROS 2 노드 포인터를 가져옵니다.
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Python 도킹 노드가 열어둔 서비스를 호출하기 위한 클라이언트 생성
    client_ = node_->create_client<std_srvs::srv::SetBool>("/toggle_docking");
}

BT::PortsList ToggleDocking::providedPorts() {
    return {
        // XML에서 <ToggleDocking enable="true" /> 형식으로 사용
        BT::InputPort<bool>("enable", true, "Enable (true) or disable (false) ArUco docking")
    };
}

BT::NodeStatus ToggleDocking::onStart() {
    bool enable_docking = true;
    if (!getInput("enable", enable_docking)) {
        RCLCPP_WARN(node_->get_logger(), "[ToggleDocking] 'enable' port missing. Defaulting to true.");
    }

    // 서비스 서버가 켜져 있는지 확인 (최대 1초 대기)
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "[ToggleDocking] Service /toggle_docking is not available. Is aruco_docker_node running?");
        return BT::NodeStatus::FAILURE;
    }

    // 서비스 요청(Request) 객체 생성 및 데이터 셋업
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable_docking;

    // 서비스 호출 (비동기 처리로 Behavior Tree의 틱(Tick)이 멈추지 않게 방지)
    future_ = client_->async_send_request(request).future.share();

    RCLCPP_INFO(node_->get_logger(), "[ToggleDocking] Requested docking mode: %s", enable_docking ? "ON" : "OFF");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ToggleDocking::onRunning() {
    // 응답이 도착했는지 확인 (블로킹 없이 0초 대기)
    if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
            auto response = future_.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "[ToggleDocking] Success: %s", response->message.c_str());
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[ToggleDocking] Failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[ToggleDocking] Service call failed with exception: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
    }

    // 응답이 아직 오지 않았다면 RUNNING을 반환하여 다음 틱에서 계속 검사함
    return BT::NodeStatus::RUNNING;
}

void ToggleDocking::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "[ToggleDocking] Node halted before service call completed.");
}

}  // namespace theimc_bt_nodes