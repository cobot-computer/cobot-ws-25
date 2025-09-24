"""
Simulates:
    - game_manager
        | /chess/game_config
        | /chess/time
        | /chess/game_state
    -control_panel
        | /cobot0/enabled
        | /cobot0/max_speed
"""

import rclpy
from rclpy.node import Node
import chess
import multiprocessing as mp
from chess_msgs.msg import FullFEN, GameConfig, ChessTime, CobotEnabled, CobotSpeed
import re


class TopicTester(Node):
    def __init__(self):
        super().__init__("topic_tester")

        # Initialize variables.
        self.board = chess.Board()
        self.game_config = GameConfig()
        self.time = ChessTime(
            white_time_left=self.game_config.time_base, black_time_left=self.game_config.time_base
        )
        self.cobot0_enabled = False
        self.cobot0_max_speed = 0.1

        # Setup publishers.
        self.game_state_pub = self.create_publisher(FullFEN, "chess/game_state", 10)
        self.game_config_pub = self.create_publisher(GameConfig, "chess/game_config", 10)
        self.time_pub = self.create_publisher(ChessTime, "chess/time", 10)
        self.cobot0_enabled_pub = self.create_publisher(CobotEnabled, "cobot0/enabled", 10)
        self.cobot0_max_speed_pub = self.create_publisher(CobotSpeed, "cobot0/max_speed", 10)

        # Setup timer to update game time.
        self.clock_timer = self.create_timer(0.1, self.clock_timer_callback)
        self.slow_timer = self.create_timer(1, self.publish_all)

        # Publish initial configuration.
        self.publish_all()

    def clock_timer_callback(self):
        if self.board.turn == chess.WHITE:
            self.time.white_time_left -= 100
            if self.time.white_time_left < 1000:
                self.time.white_time_left = 1000
        else:
            self.time.black_time_left -= 100
            if self.time.black_time_left < 1000:
                self.time.black_time_left = 1000


    def publish_all(self):
        # self.game_state_pub.publish(FullFEN(fen=self.board.fen()))
        self.game_config_pub.publish(self.game_config)
        # self.time_pub.publish(self.time)
        self.cobot0_enabled_pub.publish(CobotEnabled(enabled=self.cobot0_enabled))
        self.cobot0_max_speed_pub.publish(CobotSpeed(speed=self.cobot0_max_speed))

    def process_cmd(self, cmd):
        """
        Process a command string using regex.

        Commands:
            - "move <move>": Make a move.
            - "set <key> <value>": Set a parameter.
            - "get <key>": Get a parameter.
            - "pause": Pause the game timer.
            - "resume": Resume the game timer.
            - "exit": Exit the program.
        """
        move_match = re.match(r"move ([a-h][1-8][a-h][1-8]x?[qrbn]?)", cmd) #was [1-8]x?[a-h]
        set_match = re.match(r"set (\w+) (.+)", cmd)
        get_match = re.match(r"get (\w+)", cmd)
        if move_match:
            move = chess.Move.from_uci(move_match.group(1))
            if move in self.board.legal_moves:
                if self.board.turn == chess.WHITE:
                    self.time.white_time_left += self.game_config.time_increment
                else:
                    self.time.black_time_left += self.game_config.time_increment
                self.board.push(move)
                self.game_state_pub.publish(FullFEN(fen=self.board.fen())) #was commented out, testing adding it back in 
            else:
                print("Invalid move.")
        elif set_match:
            key = set_match.group(1)
            value = set_match.group(2)
            if key in ["white_time_left", "white", "w"]:
                self.time.white_time_left = int(value)
                self.time_pub.publish(self.time)
            elif key in ["black_time_left", "black", "b"]:
                self.time.black_time_left = int(value)
                self.time_pub.publish(self.time)
            elif key in ["time_increment", "increment", "inc"]:
                self.game_config.time_increment = int(value)
                self.game_config_pub.publish(self.game_config)
            elif key in ["enabled", "e"]:
                if value in ["true", "True", "T", "t", "1"]:
                    self.cobot0_enabled = True
                    self.cobot0_enabled_pub.publish(CobotEnabled(enabled=self.cobot0_enabled))
                elif value in ["false", "False", "F", "f", "0"]:
                    self.cobot0_enabled = False
                    self.cobot0_enabled_pub.publish(CobotEnabled(enabled=self.cobot0_enabled))
                else:
                    print("Invalid value.")
            elif key in ["max_speed", "speed", "s"]:
                self.cobot0_max_speed = float(value)
                self.cobot0_max_speed_pub.publish(CobotSpeed(speed=self.cobot0_max_speed))
            else:
                print("Invalid key.")
        elif get_match:
            key = get_match.group(1)
            if key in ["white_time_left", "white", "w"]:
                print(self.time.white_time_left)
            elif key in ["black_time_left", "black", "b"]:
                print(self.time.black_time_left)
            elif key in ["time_increment", "increment", "inc"]:
                print(self.game_config.time_increment)
            elif key in ["enabled", "e"]:
                print(self.cobot0_enabled)
            elif key in ["max_speed", "speed", "s"]:
                print(self.cobot0_max_speed)
            else:
                print("Invalid key.")
        elif cmd in ["pause", "p"]:
            self.clock_timer.cancel()
        elif cmd in ["resume", "r"]:
            self.clock_timer = self.create_timer(0.1, self.clock_timer_callback)
        elif cmd in ["exit", "quit", "q"]:
            return False
        else:
            print("Invalid command.")
        return True

    def destroy(self):
        self.clock_timer.cancel()
        super().destroy_node()


def spin(pipe):
    rclpy.init()
    node = TopicTester()

    while rclpy.ok():
        rclpy.spin_once(node)
        if pipe.poll():
            cmd = pipe.recv()
            ok = node.process_cmd(cmd)
            pipe.send(ok)
            if not ok:
                rclpy.shutdown()
                return


def main():

    # Start the node in a separate process.
    main_pipe, child_pipe = mp.Pipe()
    process = mp.Process(target=spin, args=(child_pipe,))
    process.start()

    # Process commands in the main process.
    while True:
        cmd = input("Enter a command: ")
        main_pipe.send(cmd)
        ok = main_pipe.recv()
        if not ok:
            print("Exiting.")
            break

    # Clean up.
    process.join()


if __name__ == "__main__":
    main()
