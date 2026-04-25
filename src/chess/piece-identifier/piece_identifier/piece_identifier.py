import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
import numpy as np
from .yolomodel import YOLOModel
from chess_msgs.msg import FullFEN, GameConfig, ChessTime
import time
import chess


class PieceIdentifier(Node):
    def __init__(self):
        super().__init__('piece_identifier')

        self.declare_parameter(
            'initial_game_state',
            chess.STARTING_FEN,
            ParameterDescriptor(description='FEN string describing the initial game state'),
        )
        self.declare_parameter(
            'detect_interval',
            0.5,
            ParameterDescriptor(description='Minimum seconds between detection runs'),
        )

        self.model = YOLOModel('cobot_best.pt')
        self.br = CvBridge()

        self.published_board = chess.Board(self.get_parameter('initial_game_state').value)

        self.board_img_sub = self.create_subscription(
            Image, 'chessboard/image_raw', self.board_img_cb, 10)
        self.restart_game_sub = self.create_subscription(
            GameConfig, 'chess/restart_game', self.restart_game_cb, 10)
        self.clock_time_sub = self.create_subscription(
            ChessTime, 'chess/time', self.clock_time_cb, 10)

        self.game_state_pub = self.create_publisher(FullFEN, 'chess/game_state', 10)
        self.image_publisher = self.create_publisher(Image, '/chessboard/annotated/image_raw', 10)

        self.game_state_pub.publish(FullFEN(fen=self.published_board.fen()))

        # Clock state
        self._white_to_play = None
        self._pending_wtp = None
        self._pending_wtp_since = 0.0

        # Detection state
        self._armed = False
        self._armed_at = 0.0
        self._stable_result = None   # (board, fen, uci)
        self._stable_since = 0.0
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_detect_time = 0.0

    def restart_game_cb(self, _):
        self.published_board = chess.Board(self.get_parameter('initial_game_state').value)
        self.game_state_pub.publish(FullFEN(fen=self.published_board.fen()))
        self._disarm()

    def _arm(self):
        self._armed = True
        self._armed_at = time.time()
        self._stable_result = None
        self._stable_since = 0.0
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_detect_time = 0.0
        self.get_logger().info('Clock pressed — detection armed')

    def _disarm(self):
        self._armed = False
        self._stable_result = None
        self._stable_since = 0.0
        self._stable_count = 0

    def clock_time_cb(self, msg: ChessTime):
        new_wtp = msg.white_to_play
        now = time.monotonic()

        if self._white_to_play is None:
            self._white_to_play = new_wtp
            return

        if self._armed:
            return

        # Debounce: require the new value to hold for 1 s before committing
        if new_wtp == self._white_to_play:
            self._pending_wtp = None
            return

        if self._pending_wtp != new_wtp:
            self._pending_wtp = new_wtp
            self._pending_wtp_since = now
            return

        if now - self._pending_wtp_since < 1.0:
            return

        self._white_to_play = new_wtp
        self._pending_wtp = None

        # Arm when either player presses the clock (their turn ends)
        self._arm()

    def _get_occupancy(self, inference_matrix):
        """Return the set of squares the camera sees as occupied."""
        occupied = set()
        for y in range(8):
            for x in range(8):
                label = max(inference_matrix[y][x], key=inference_matrix[y][x].get)
                if label != 'empty':
                    occupied.add(chess.square(x, 7 - y))
        return occupied

    def _detect_move(self, inference_matrix):
        """
        Compare camera occupancy against published_board occupancy.
        Returns (board, fen, uci) on success, (None, None, None) on failure.
        """
        pb = self.published_board
        prev_occ = {sq for sq in chess.SQUARES if pb.piece_at(sq) is not None}
        curr_occ = self._get_occupancy(inference_matrix)

        vacated = prev_occ - curr_occ   # were filled, now empty
        arrived = curr_occ - prev_occ   # were empty, now filled

        self.get_logger().info(
            f'occupancy diff — '
            f'vacated: {sorted(chess.square_name(s) for s in vacated)}  '
            f'arrived: {sorted(chess.square_name(s) for s in arrived)}'
        )

        # Best-fit: find the legal move whose expected occupancy change
        # best matches what the camera sees (fewest unexplained squares).
        best_moves = []
        best_noise = float('inf')

        for m in pb.legal_moves:
            test = pb.copy()
            test.push(m)
            test_occ = {sq for sq in chess.SQUARES if test.piece_at(sq) is not None}

            exp_vacated = prev_occ - test_occ
            exp_arrived = test_occ - prev_occ

            noise = (len(vacated.symmetric_difference(exp_vacated))
                     + len(arrived.symmetric_difference(exp_arrived)))

            if noise < best_noise:
                best_noise = noise
                best_moves = [m]
            elif noise == best_noise:
                best_moves.append(m)

        MAX_NOISE = 2
        if not best_moves or best_noise > MAX_NOISE:
            return None, None, None

        if len(best_moves) == 1:
            move = best_moves[0]
        else:
            # Ambiguous — only resolvable if all candidates share from/to (promotion variants)
            from_sqs = {m.from_square for m in best_moves}
            to_sqs = {m.to_square for m in best_moves}
            if len(from_sqs) != 1 or len(to_sqs) != 1:
                return None, None, None
            queen_promos = [m for m in best_moves if m.promotion == chess.QUEEN]
            if not queen_promos:
                return None, None, None
            move = queen_promos[0]

        result_board = pb.copy()
        result_board.push(move)
        return result_board, result_board.fen(), move.uci()

    def board_img_cb(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        pil_image = PILImage.fromarray(cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))
        preview_image, inference_matrix = self.model.crop_and_run(pil_image)

        self.image_publisher.publish(
            self.br.cv2_to_imgmsg(np.array(preview_image), 'rgb8'))

        if not self._armed:
            return

        now = time.time()
        detect_interval = self.get_parameter('detect_interval').value
        if now - self._last_detect_time < detect_interval:
            return
        self._last_detect_time = now

        elapsed = now - self._armed_at

        if elapsed > 60.0:
            self.get_logger().warn('Detection timed out — no stable move found')
            self._disarm()
            return

        candidate_board, candidate_fen, candidate_uci = self._detect_move(inference_matrix)

        # Stability tracking: require the same result for 2 s and at least 2 frames.
        # Noisy frames (None) are ignored rather than resetting the timer.
        prev_fen = self._stable_result[1] if self._stable_result else None
        if candidate_fen is None:
            pass
        elif candidate_fen != prev_fen:
            self._stable_result = (candidate_board, candidate_fen, candidate_uci)
            self._stable_since = now
            self._stable_count = 0
        else:
            self._stable_count += 1

        if now - self._last_log_time >= 3.0:
            self._last_log_time = now
            if candidate_fen is None:
                self.get_logger().info(f'[{elapsed:.0f}s] no move detected yet')
            else:
                self.get_logger().info(
                    f'[{elapsed:.0f}s] candidate {candidate_uci} '
                    f'stable {now - self._stable_since:.1f}s ({self._stable_count} frames)'
                )

        if candidate_fen is None:
            return

        if now - self._stable_since < 2.0 or self._stable_count < 2:
            return

        # Committed
        result_board, _, uci = self._stable_result
        self._disarm()
        self.published_board = result_board
        self.get_logger().info(f'Move committed: {uci}')
        self.game_state_pub.publish(FullFEN(fen=result_board.fen()))


def main(args=None):
    rclpy.init(args=args)
    node = PieceIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
