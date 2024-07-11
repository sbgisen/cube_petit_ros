#include <rclcpp/rclcpp.hpp>
#include "M2006Ros2.hpp"  // M2006Ros2 のヘッダファイルをインクルード

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // auto m2006 = std::make_shared<dji_ros_controller::M2006Ros2>();
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_manager");

  // auto dji_can_node = m2006->getDjiCanNode();

  rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(m2006->get_node_base_interface());
  executor.add_node(lifecycle_manager->get_node_base_interface());

  // Trigger the transition to configure state
  // m2006->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Start the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
