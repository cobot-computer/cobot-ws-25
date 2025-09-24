#include <rclcpp/rclcpp.hpp>
#include "chess_player_breakdown/chess_player_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChessPlayerNode>();
  rclcpp::Rate loop_rate(1);

  RCLCPP_INFO(node->get_logger(), "Starting chess_player_breakdown...");

  while (rclcpp::ok()) {
    node->take_turn_();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}