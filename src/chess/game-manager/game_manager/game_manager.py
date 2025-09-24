import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
import serial
import chess
import re
from datetime import datetime, timedelta
from chess_msgs.msg import FullFEN, PartialFEN, GameConfig, ChessTime
from chess_msgs.srv import SetTime, SetGameState, RestartGame


def parse_time_string(time_string):
    """
    Parse a duration string into milliseconds. The string can be in any of the following
    formats (evaluated in order):
        - MM:SS
        - MM:SS.SSS
        - HH:MM:SS
        - HH:MM:SS.SSS
        - raw number of milliseconds

    :param time_string: The time string to parse
    :return: The duration in milliseconds
    """

    TIME_FORMATS = [
        "%M:%S",
        "%M:%S.%f",
        "%H:%M:%S",
        "%H:%M:%S.%f",
    ]

    for format in TIME_FORMATS:
        try:
            dt = datetime.strptime(time_string, format)
            delta = timedelta(
                hours=dt.hour,
                minutes=dt.minute,
                seconds=dt.second,
                microseconds=dt.microsecond,
            )
            return delta.total_seconds() * 1000
        except ValueError:
            pass
    else:
        try:
            return float(time_string)
        except ValueError:
            raise ValueError(f"Invalid time string: {time_string}")


class GameManagerNode(Node):
    def __init__(self):
        super().__init__("game_manager")

        # Declare parameters.
        self.declare_parameter(
            "clock_port",
            "/dev/ttyChessClock0",
            ParameterDescriptor(description="Path to the clock serial port"),
        )
        self.declare_parameter(
            "initial_game_state",
            chess.STARTING_FEN,
            ParameterDescriptor(description="FEN string describing the initial game state"),
        )
        self.declare_parameter(
            "time_base",
            "5:00",
            ParameterDescriptor(description="Base time for the game"),
        )
        self.declare_parameter(
            "time_increment",
            "0:05",
            ParameterDescriptor(description="Time increment for each move"),
        )

        # Try to connect to the clock. If it fails, log an error and continue.
        self._clock_port = self.get_parameter("clock_port").value
        try:
            self._clock_connection = serial.Serial(self._clock_port, 9600)
        except serial.SerialException:
            self.get_logger().error(f"Could not connect to chess clock at {self._clock_port}")
            self._clock_connection = None

        # Initialize the game.
        self._board = chess.Board(self.get_parameter("initial_game_state").value)
        self._time_base = parse_time_string(self.get_parameter("time_base").value)
        self._time_increment = parse_time_string(self.get_parameter("time_increment").value)
        self._white_time_left = self._time_base
        self._black_time_left = self._time_base

        # Subscribe to the board state.
        self._board_state_sub = self.create_subscription(
            PartialFEN,
            "chess/board_state",
            self.board_state_callback,
            10,
        )

        # Create publishers.
        self._game_config_pub = self.create_publisher(
            GameConfig,
            "chess/game_config",
            10,
        )
        self._time_pub = self.create_publisher(
            ChessTime,
            "chess/time",
            10,
        )
        self._game_state_pub = self.create_publisher(
            FullFEN,
            "chess/game_state",
            10,
        )

        # Publish the game configuration. This happens at the beginning of the game and remains
        # constant until the game is restarted.
        self._game_config_pub.publish(
            GameConfig(
                time_base=self._time_base,
                time_increment=self._time_increment,
            )
        )

        # Create service servers.
        self._set_time_srv = self.create_service(
            SetTime,
            "chess/set_time",
            self.set_time_callback,
        )
        self._set_game_state_srv = self.create_service(
            SetGameState,
            "chess/set_game_state",
            self.set_game_state_callback,
        )
        self._restart_game_srv = self.create_service(
            RestartGame,
            "chess/restart_game",
            self.restart_game_callback,
        )

        # Create timer for periodically polling the clock.
        self._clock_timer = self.create_timer(0.5, self.clock_timer_callback)  # 2 Hz
        self._acks_received = 0  # The number of unused acks that have been received

    def destroy(self):
        if self._clock_connection is not None:
            self._clock_connection.close()
        super().destroy_node()

    def get_clock_connection(self):
        """
        Gets the connection to the chess clock. If the connection is `None`, attempts to reconnect
        to the clock.
        """
        if self._clock_connection is None:
            self.get_logger().warn("Clock is not connected; attempting to reconnect")
            try:
                self._clock_connection = serial.Serial(self._clock_port, 9600)
            except serial.SerialException:
                self.get_logger().error(f"Could not connect to chess clock at {self._clock_port}")
                self._clock_connection = None

        return self._clock_connection

    def poll_clock(self):
        """
        Polls the clock for a new message.

        :return: The message received from the clock, or `None`
        """
        if self.get_clock_connection() is None:
            self.get_logger().error("Clock is not connected; cannot poll")
            return None
        msg = self._clock_connection().readline()
        if not "\n" in msg:
            self.get_logger().warn("Clock timed out")
            return None
        if msg.strip() == "ack":
            self._acks_received += 1

    def wait_for_ack(self):
        """
        Waits for an acknowledgement from the clock. If no acknowledgement is received, logs an
        error and returns False.
        """
        if self.get_clock_connection() is None:
            self.get_logger().error("Clock is not connected")
            return False
        while self._acks_received < 1:
            if self.poll_clock() is None:
                self.get_logger().error("Did not receive ack in time")
                return False
        self._acks_received -= 1
        return True

    def board_state_callback(self, msg):
        """
        Callback for when a new board state is received. A board state is a partial FEN string
        containing only the piece positions. This function will determine which legal move, if any,
        was made and update the game state accordingly.

        :param msg: The partial FEN message containing the board state
        """

        # Parse the partial FEN string into a board object.
        new_board = chess.Board(msg.fen)
        current_partial_fen = self._board.board_fen()
        new_partial_fen = new_board.board_fen()

        # Find all squares that have changed.
        changed_squares = []
        for square in chess.SQUARES:
            if new_board.piece_at(square) != self._game_state.piece_at(square):
                changed_squares.append(square)
        num_changed_squares = len(changed_squares)

        # If there are no changed squares, then no move was made.
        if num_changed_squares == 0:
            self.get_logger().warn(f"Received board update with no changes: {current_partial_fen}")
            return

        # We want to use the `find_move` function to determine which move was made. This function
        # requires the origin and destination squares of the move. The method for determining these
        # depends on whether a regular move or castling move was made.
        origin_square = None
        destination_square = None

        # If there are two changed squares, then a regular move was made. One square will be the
        # origin of the move and the other will be the destination. The origin square must always be
        # empty and the destination square must always be occupied. If this is not the case, then
        # the move is invalid.
        if num_changed_squares == 2:
            s0_empty = self._board.piece_at(changed_squares[0]) is None
            s1_empty = self._board.piece_at(changed_squares[1]) is None
            if s0_empty and not s1_empty:
                origin_square = changed_squares[0]
                destination_square = changed_squares[1]
            elif not s0_empty and s1_empty:
                origin_square = changed_squares[1]
                destination_square = changed_squares[0]
            else:
                self.get_logger().error(
                    f"Invalid move detected. One square must be empty and the other must be occupied, but this is not the case: {current_partial_fen} -> {new_partial_fen}"
                )
                return

        # If there are three changed squares, then en passant has occurred. The origin square will
        # have a pawn of the same color as the destination square. The destination square will be
        # be the only one containing a piece. If this is not the case, then the move is invalid.
        elif num_changed_squares == 3:
            dest_color = None
            for square in changed_squares:
                if new_board.piece_at(square) is not None:
                    if dest_color is not None:
                        self.get_logger().error(
                            f"Invalid move detected. En passant move detected, but two destination squares were found: {current_partial_fen} -> {new_partial_fen}"
                        )
                        return
                    destination_square = square
                    dest_color = new_board.piece_at(destination_square).color
                    break
            if dest_color is None:
                self.get_logger().error(
                    f"Invalid move detected. En passant move detected, but no destination square was found: {current_partial_fen} -> {new_partial_fen}"
                )
                return
            for square in changed_squares:
                if self._board.piece_at(square).color == dest_color:
                    if origin_square is not None:
                        self.get_logger().error(
                            f"Invalid move detected. En passant move detected, but two origin squares were found: {current_partial_fen} -> {new_partial_fen}"
                        )
                        return
                    origin_square = square
                    break

        # If there are four changed squares, then castling has occurred. A king will always move
        # when castling, and the `find_move` function requires the king's origin and destination
        # squares (not the rook's). Therefore, as long as one changed square in the current board
        # state is a king, we can use that as the origin square, and the same logic can be applied
        # to the destination square. If this is not the case, then the move is invalid.
        elif num_changed_squares == 4:
            for square in changed_squares:
                if self._board.piece_at(square).piece_type == chess.KING:
                    if origin_square is None:
                        origin_square = square
                    else:
                        self.get_logger().error(
                            f"Invalid move detected. Castling move detected, but two kings were found: {current_partial_fen} -> {new_partial_fen}"
                        )
                        return
                elif new_board.piece_at(square).piece_type == chess.KING:
                    if destination_square is None:
                        destination_square = square
    // Check if the timeout has been exceeded.
    if (timeout > 0) {
      auto elapsed = steady_clock::now() - start_time;
      if (duration_cast<milliseconds>(elapsed).count() > timeout) {
        RCLCPP_WARN(logger, "Timeout exceeded");
        throw runtime_error("Timeout exceeded");
      }
                    else:
                        self.get_logger().error(
                            f"Invalid move detected. Castling move detected, but two kings were found: {current_partial_fen} -> {new_partial_fen}"
                        )
                        return

    // Check if the timeout has been exceeded.
    if (timeout > 0) {
      auto elapsed = steady_clock::now() - start_time;
      if (duration_cast<milliseconds>(elapsed).count() > timeout) {
        RCLCPP_WARN(logger, "Timeout exceeded");
        throw runtime_error("Timeout exceeded");
      }
        # We record the new piece at the destination square in case a pawn promotion occurred. If
        # there is no promotion, `find_move` will ignore the parameter (TODO: verify this), so we
        # can simply pass this in every time.
        promoted_piece_type = new_board.piece_at(destination_square).piece_type

        # Find the move that was made.
        try:
            move = self._board.find_move(
                origin_square,
                destination_square,
                promoted_piece_type,
            )
        except chess.IllegalMoveError:
            self.get_logger().error(
                f"Invalid move detected. No legal move exists from {origin_square} to {destination_square}: {current_partial_fen} -> {new_partial_fen}"
            )
            return

        # Make the move and publish the new game state.
        self._board.push(move)
        self._game_state_pub.publish(FullFEN(fen=self._board.fen()))

    def set_time_callback(self, request, response):
        """
        Callback for when a new time is requested. This function will set the time on the clock
        and publish the new time to the `chess/time` topic.

        :param request: The request message containing the new time
        :param response: The response message
        """

        self.get_logger().info(
            f"Setting game time to W:{request.time.white_time_left} B:{request.time.black_time_left}"
        )

        if self.get_clock_connection() is None:
            self.get_logger().error("Clock is not connected")
            response.success = False
            return response

        cmd = f"set w {request.time.white_time_left} b {request.time.black_time_left}\n"
        self._clock_connection.write(cmd)
        if not self.wait_for_ack():
            self.get_logger().error(f"Clock did not acknowledge command `{cmd}`")
            response.success = False
            return response

        self._time_pub.publish(request.time)
        response.success = True
        return response

    def set_game_state_callback(self, request, response):
        """
        Callback for when a new game state is requested. This function will set the game state
        and publish the new game state to the `chess/game_state` topic.

        :param request: The request message containing the new game state
        :param response: The response message
        """

        self.get_logger().info(f"Setting game state to {request.fen}")
        self._board = chess.Board(request.fen)
        self._game_state_pub.publish(request.fen)
        response.success = True
        return response

    def restart_game_callback(self, request, response):
        """
        Callback for when the game is restarted. This function will reset the game state and
        publish the new game state to the `chess/game_state` topic.

        :param request: The request message
        :param response: The response message
        """

        new_fen = request.initial_fen if request.initial_fen != "" else chess.STARTING_FEN
        new_time_base = parse_time_string(request.config.time_base)
        new_time_increment = parse_time_string(request.config.time_increment)
        self.get_logger().info(
            f"Restarting game with game_state={new_fen}, time_base={new_time_base}, time_increment={new_time_increment}"
        )

        if self.self.get_clock_connection() is None:
            self.get_logger().error("Clock is not connected")
            response.success = False
            return response

        cmd = f"rst {request.config.time_base} {request.config.time_increment}\n"
        self._clock_connection.write(cmd)
        if not self.wait_for_ack():
            self.get_logger().error(f"Clock did not acknowledge command `{cmd}`")
            response.success = False
            return response

        self._white_time_left = new_time_base
        self._black_time_left = new_time_base
        self._time_pub.publish(
            ChessTime(
                white_time_left=self._white_time_left,
                black_time_left=self._black_time_left,
            )
        )

        self._board = chess.Board(new_fen)
        self._time_base = new_time_base
        self._time_increment = new_time_increment
        self._game_config_pub.publish(request.config)
        self._game_state_pub.publish(new_fen)
        response.success = True
        return response

    def clock_timer_callback(self):
        """
        Callback for the clock timer. This function will poll the clock and update the game state
        accordingly.
        """

        if self.self.get_clock_connection() is None:
            return

        msg = self.poll_clock()
        if msg is None:
            self.get_logger().warn("Could not poll clock")
            return

        m = re.match(r"time w (\d+) b (\d+)", msg.strip())
        if m:
            self._white_time_left = int(m.group(1))
            self._black_time_left = int(m.group(2))
            self._time_pub.publish(
                ChessTime(
                    white_time_left=self._white_time_left,
                    black_time_left=self._black_time_left,
                )
            )


def main(args=None):
    rclpy.init(args=args)

    game_manager_node = GameManagerNode()
    rclpy.spin(game_manager_node)
    game_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
