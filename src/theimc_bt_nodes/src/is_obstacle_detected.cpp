#include "theimc_bt_nodes/is_obstacle_detected.hpp"
#include <cmath>

namespace theimc_bt_nodes
{

IsObstacleDetected::IsObstacleDetected(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(xml_tag_name, config)
{
  // 블랙보드에서 ROS 2 노드 포인터를 가져옵니다.
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("Missing Node in blackboard");
  }

  // 라이다 센서 데이터(/scan) 구독 설정
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&IsObstacleDetected::scan_callback, this, std::placeholders::_1));
}

BT::PortsList IsObstacleDetected::providedPorts()
{
  return {
    BT::InputPort<double>("max_distance", 0.5, "Maximum distance to detect an obstacle in meters"),
    BT::InputPort<std::string>("direction", "front", "Check direction: 'front' or 'rear'")
  };
}

void IsObstacleDetected::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 멀티스레드 환경에서 데이터가 꼬이지 않도록 Mutex 잠금
  std::lock_guard<std::mutex> lock(mutex_);
  latest_scan_ = msg;
}

BT::NodeStatus IsObstacleDetected::tick()
{
  double max_distance = 0.5;
  std::string direction = "front";

  getInput("max_distance", max_distance);
  getInput("direction", direction);

  sensor_msgs::msg::LaserScan::SharedPtr current_scan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_scan = latest_scan_;
  }

  // 아직 라이다 데이터가 한 번도 들어오지 않았다면 장애물이 없다고 간주 (FAILURE)
  if (!current_scan) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsObstacleDetected] No scan data received yet.");
    return BT::NodeStatus::FAILURE;
  }

  double fov_rad = 45.0 * M_PI / 180.0;  // 45 degrees in radians
  
  // 수정된 부분: size_t i = 0 으로 오타 수정
  for (size_t i = 0; i < current_scan->ranges.size(); ++i) {
    float range = current_scan->ranges[i];

    if (std::isnan(range) || std::isinf(range) || 
        range < current_scan->range_min || range > max_distance) {
      continue;  // 무한대(inf)나 쓰레기값(NaN), 범위 밖은 무시
    }

    double angle = current_scan->angle_min + i * current_scan->angle_increment;
    angle = std::atan2(std::sin(angle), std::cos(angle));  // Normalize angle to [-pi, pi]

    bool in_range = false;
    if (direction == "front") {
        if (angle >= -fov_rad && angle <= fov_rad) {
            in_range = true;
        }
    }
    else if (direction == "rear") {
        if (angle >= M_PI - fov_rad || angle <= -M_PI + fov_rad) {
            in_range = true;
        }
    }
    else {
        in_range = true; // 방향 설정이 없으면 전체 감시
    }

    if (in_range) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000, // 로그 너무 많이 뜨지 않게 1초로 조절
            "[IsObstacleDetected] Obstacle detected at distance: %.2f meters, angle: %.2f radians",
            range, angle);
        return BT::NodeStatus::SUCCESS;  // 장애물이 발견되면 SUCCESS 반환
    }
  }

  // 반복문을 다 돌았는데도 장애물이 없다면 안전함 (FAILURE)
  return BT::NodeStatus::FAILURE;
}

}  // namespace theimc_bt_nodes