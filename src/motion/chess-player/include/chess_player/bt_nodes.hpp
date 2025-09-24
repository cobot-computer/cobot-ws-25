#ifndef BT_NODES_HPP
#define BT_NODES_HPP

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <string>
#include "chess_player/chess_player_node.hpp"
#include "chess_msgs/msg/full_fen.hpp"

using std::string;

// ----------- Turn Setup -------------
class TurnSetupAction : public BT::SyncActionNode {
public:
    TurnSetupAction(const string& name, const BT::NodeConfiguration& config,
                    std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        if (chess_node_->turn_setup()) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Find Best Move -------------
class FindBestMoveAction : public BT::SyncActionNode {
public:
    FindBestMoveAction(const string& name, const BT::NodeConfiguration& config,
                       std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        return chess_node_->find_best_move_()
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Parse Move -------------
class ParseMoveAction : public BT::SyncActionNode {
public:
    ParseMoveAction(const string& name, const BT::NodeConfiguration& config,
                    std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        string move;
        getInput("move", move);
        return chess_node_->parse_move_(move)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<string>("move"), BT::OutputPort<libchess::Move>("parsed_move")};
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Capture Piece -------------
class CapturePieceAction : public BT::SyncActionNode {
public:
    CapturePieceAction(const string& name, const BT::NodeConfiguration& config,
                       std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        string move;
        getInput("parsed_move", move);
        return chess_node_->capture_piece_(move)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<string>("parsed_move")};
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Move Piece -------------
class MovePieceAction : public BT::SyncActionNode {
public:
    MovePieceAction(const string& name, const BT::NodeConfiguration& config,
                    std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        string move;
        getInput("parsed_move", move);
        return chess_node_->move_piece_(move)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<string>("parsed_move")};
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Hit Clock -------------
class HitClockAction : public BT::SyncActionNode {
public:
    HitClockAction(const string& name, const BT::NodeConfiguration& config,
                   std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        return chess_node_->hit_clock_()
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

// ----------- Move Home -------------
class MoveHomeAction : public BT::SyncActionNode {
public:
    MoveHomeAction(const string& name, const BT::NodeConfiguration& config,
                   std::shared_ptr<ChessPlayerNode> chess_node)
        : BT::SyncActionNode(name, config), chess_node_(chess_node) {}

    BT::NodeStatus tick() override {
        return chess_node_->move_home_()
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<ChessPlayerNode> chess_node_;
};

#endif  // BT_NODES_HPP

