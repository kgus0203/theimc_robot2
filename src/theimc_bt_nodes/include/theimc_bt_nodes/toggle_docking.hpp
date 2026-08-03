#ifndef THEIMC_BT_NODES__TOGGLE_DOCKING_HPP_
#define THEIMC_BT_NODES__TOGGLE_DOCKING_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <future>

namespace theimc_bt_nodes {

class ToggleDocking : public BT::StatefulActionNode {
public:
    ToggleDocking(const std::string& name, const BT::NodeConfiguration& config);

    // XML에서 받을 입력 포트 정의
    static BT::PortsList providedPorts();

    // StatefulActionNode의 3가지 필수 오버라이드 함수
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    
    // 서비스 응답을 비동기로 기다리기 위한 future 객체
    std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr> future_;
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__TOGGLE_DOCKING_HPP_