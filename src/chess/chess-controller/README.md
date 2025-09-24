# Chess Controller

This package contains a ROS2 node that interfaces with a UCI-compatible chess engine to play chess.
It contains an action server that accepts a board state and clock time, and returns a move.

The action is defined in `chess_msgs/action/FindBestMove.action`.