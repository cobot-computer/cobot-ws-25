import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from chess_msgs.msg import GameConfig
from chess_msgs.action import FindBestMove

import threading

import chess
import chess.engine


class ChessEngineActionServer(Node):
    def __init__(self):
        super().__init__("chess_controller")

        # Declare parameters
        self.declare_parameter(
            "engine_path",
            "stockfish",
            ParameterDescriptor(description="Path to the chess engine executable"),
        )

        # Subscribe to the game configuration topic
        self._current_game_config = None
        self._game_config_sub = self.create_subscription(
            GameConfig,
            "chess/game_config",
            lambda cfg: setattr(self, "_current_game_config", cfg),
            10,
        )

        # Only a single goal can be active at a time
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        # Start the chess engine process
        self._engine = chess.engine.SimpleEngine.popen_uci(self.get_parameter("engine_path").value)

        # Create action server for finding the best move
        self._action_server = ActionServer(
            self,
            FindBestMove,
            "chess/find_best_move",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Chess engine action server is up")

    def destroy(self):
        self._transport.close()
        self._asyncio_loop.close()
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info("Received goal request")

        if self._current_game_config is None:
            self.get_logger().error("The `game_configuration` topic has not been published to yet")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Start execution of a goal."""
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        self.get_logger().info("Starting execution of goal")
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")

        if goal_handle.request.analysis_mode:
            self.get_logger().info("Cancelling analysis mode")
            return CancelResponse.ACCEPT
        else:
            self.get_logger().warn("Cannot cancel play mode")
            return CancelResponse.REJECT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        game_config = self._current_game_config
        if game_config is None:
            self.get_logger().error(
                "Best move requested without any data from the `game_configuration` topic`"
            )
            goal_handle.abort()
            return FindBestMove.Result()

        board_fen = goal_handle.request.fen.fen
        remaining_times = goal_handle.request.time

        board = chess.Board(board_fen)
        limit = chess.engine.Limit(
            white_clock=remaining_times.white_time_left / 1000,
            black_clock=remaining_times.black_time_left / 1000,
            white_inc=game_config.time_increment / 1000,
            black_inc=game_config.time_increment / 1000,
        )

        # Analysis mode allows cancellation but not drawing or resigning
        if goal_handle.request.analysis_mode:
            self.get_logger().info("Executing in analysis mode")
            analysis = self._engine.analysis(board, limit=limit)
            while True:
                # Check if the goal has been aborted
                if not goal_handle.is_active:
                    self.get_logger().info("Goal aborted")
                    return FindBestMove.Result()

                # Check if the goal has been cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    return FindBestMove.Result()

                # Wait for the next info from the engine and break if a move is found
                info = analysis.next()
                if info is None:
                    break

                # Send feedback to the client
                for key, value in info.items():
                    feedback_result = FindBestMove.Feedback()
                    feedback_result.info.timestamp = self.get_clock().now().to_msg()
                    feedback_result.info.type = key
                    feedback_result.info.value = str(value)
                    goal_handle.publish_feedback(feedback_result)

            # Send the result to the client
            engine_move = analysis.wait().move
            if engine_move is None:
                self.get_logger().error("No move found")
                goal_handle.abort()
                return FindBestMove.Result()
            else:
                self.get_logger().info("Found best move")
                goal_handle.succeed()
                result = FindBestMove.Result()
                result.move.move = engine_move.uci()
                result.move.draw = Falsedriver of the
                resul_result.draw_offered:
                result.move.draw = True
                result.move.resign = False
            elif engine_result.resigned:
                result.move.draw = False
                result.move.resign = True
            elif engine_result.move is not None:
                result.move.draw = False
                result.move.resign = False
                result.move.move = engine_result.move.uci()
            else:
                self.get_logger().error("No move found")
                goal_handle.abort()
                return FindBestMove.Result()

            self.get_logger().info("Move found")
            goal_handle.succeed()
            return result


def main(args=None):
    rclpy.init(args=args)

    action_server = ChessEngineActionServer()

    rclpy.spin(action_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
