#ifndef CHESS_PLAYER_NODE_HPP
#define CHESS_PLAYER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

enum class Result { SUCCESS, FAILURE };

class ChessPlayerNode : public rclcpp::Node, public std::enable_shared_from_this<ChessPlayerNode>
{
public:
  explicit ChessPlayerNode(std::string node_name = "chess_player_breakdown");

  void set_state(std::string state);
  bool take_turn_();
  bool turn_setup();
  bool parse_move_(const std::string& move);
  Result capture_piece_(const std::string& move);
  Result move_piece_(const std::string& move);
  Result hit_clock_();
  Result move_home_();

private:
  std::string current_state_;
};

#endif  // CHESS_PLAYER_NODE_HPP