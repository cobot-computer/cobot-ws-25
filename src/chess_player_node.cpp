#include "chess_player_breakdown/chess_player_node.hpp"

ChessPlayerNode::ChessPlayerNode(std::string node_name)
: Node(node_name), current_state_("IDLE")
{
  RCLCPP_INFO(this->get_logger(), "ChessPlayerNode initialized.");
}

void ChessPlayerNode::set_state(std::string state)
{
  current_state_ = state;
  RCLCPP_INFO(this->get_logger(), "State changed to: %s", state.c_str());
}

bool ChessPlayerNode::take_turn_()
{
  RCLCPP_INFO(this->get_logger(), "Taking turn...");
  return true;
}

bool ChessPlayerNode::turn_setup()
{
  RCLCPP_INFO(this->get_logger(), "Turn setup complete.");
  return true;
}

bool ChessPlayerNode::parse_move_(const std::string& move)
{
  RCLCPP_INFO(this->get_logger(), "Parsing move: %s", move.c_str());
  return true;
}

Result ChessPlayerNode::capture_piece_(const std::string& move)
{
  RCLCPP_INFO(this->get_logger(), "Capturing piece at: %s", move.c_str());
  return Result::SUCCESS;
}

Result ChessPlayerNode::move_piece_(const std::string& move)
{
  RCLCPP_INFO(this->get_logger(), "Moving piece using move string: %s", move.c_str());
  return Result::SUCCESS;
}

Result ChessPlayerNode::hit_clock_()
{
  RCLCPP_INFO(this->get_logger(), "Hitting clock.");
  return Result::SUCCESS;
}

Result ChessPlayerNode::move_home_()
{
  RCLCPP_INFO(this->get_logger(), "Returning to home position.");
  return Result::SUCCESS;
}