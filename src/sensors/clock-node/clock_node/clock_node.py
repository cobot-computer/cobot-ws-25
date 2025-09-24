import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
import serial
import re
import time
from datetime import datetime, timedelta
from chess_msgs.msg import GameConfig, ChessTime, ClockButtons
from chess_msgs.srv import SetTime, RestartClock

#import sys 
#print("CLOCK_NODE is running from: ", __file__, file=sys.stderr)

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
            return int(delta.total_seconds() * 1000)
        except ValueError:
            pass
    else:
        try:
            return int(time_string)
        except ValueError:
            raise ValueError(f"Invalid time string: {time_string}")


class ClockNode(Node):
    def __init__(self):
        super().__init__("clock_node")
        self._acks_received = 0  # The number of unused acks that have been received

        # Declare parameters.
        self.declare_parameter(
            "clock_port",
            "/dev/ttyACM0",
            ParameterDescriptor(description="Path to the clock serial port"),
        )
        self.declare_parameter(
            "time_base",
            "5:00",
            ParameterDescriptor(description="Base time for the game"),
        )
        self.declare_parameter(
            "time_increment",
            "1:00",
            ParameterDescriptor(description="Time increment for each move"),
        )

        # Try to connect to the clock. If it fails, log an error and continue.
        self._clock_port = self.get_parameter("clock_port").value
        try:
            self._clock_connection = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=2) #changed from 0 to 2 because of some debug results.  baud 115200 could work? original 9600 
            time.sleep(5)
            self.get_logger().info("Clock connected")
        except serial.SerialException:
            self.get_logger().error(f"Could not connect to chess clock at {self._clock_port}")
            self._clock_connection = None

        # Initialize the clock.
        self._time_base = parse_time_string(self.get_parameter("time_base").value)
        self._time_increment = parse_time_string(self.get_parameter("time_increment").value)
        self._white_time_left = self._time_base
        self._black_time_left = self._time_base
        self._paused = False
        self._active = False
        self._white_btn_pressed = False
        self._black_btn_pressed = False

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
        self._buttons_pub = self.create_publisher(
            ClockButtons,
            "chess/clock_buttons",
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
        self._restart_game_srv = self.create_service(
            RestartClock,
            "chess/restart_game",
            self.restart_game_callback,
        )

        self.restart(self._time_base, self._time_increment)

        # Create timer for periodically polling the clock.
        self._clock_timer = self.create_timer(0.01, self.clock_timer_callback)  # 100 Hz-- i am trying slowing it down 

    def destroy(self):
        if self._clock_connection is not None:
            self._clock_connection.close()
        super().destroy_node()

    def poll_clock(self):
        """
        Polls the clock for a new message.

        :return: The message received from the clock, or `None`
        """
        if self._clock_connection is None:
            self.get_logger().error("Clock is not connected; cannot poll")
            return None
        msg = str(self._clock_connection.readline())
        msg = msg.replace('\r\n', '\n')
        # if not '\n' in msg:
        #     self.get_logger().warn("Clock timed out")
        #     return None
        if msg.strip() == "ack":
            self._acks_received += 1
        else:
            self.handle_message(msg)
        return msg

    def wait_for_ack(self):
        """
        Waits for an acknowledgement from the clock. If no acknowledgement is received, logs an
        error and returns False.
        """
        if self._clock_connection is None:
            self.get_logger().error("Clock is not connected")
            return False
        while self._acks_received < 1:
            msg = self.poll_clock()
            if msg is None:
                self.get_logger().error("Did not receive ack in time")
                return False
        self._acks_received -= 1
        return True

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

        if self._clock_connection is None:
            self.get_logger().error("Clock is not connected")
            response.success = False
            return response

        cmd = f"set w {request.time.white_time_left} b {request.time.black_time_left}\n"
        self.get_logger().info(f"Writing {cmd}")
        written = self._clock_connection.write(bytes(cmd, encoding="utf-8"))
        if written != len(cmd):
            self.get_logger().error(
                f"Could not write to clock. Wrote {written} bytes of {len(cmd)}"
            )
            response.success = False
            return response
        if not self.wait_for_ack():
            self.get_logger().error(f"Clock did not acknowledge command `{cmd}`")
            response.success = False
            return response

        self._time_pub.publish(request.time)
        response.success = True
        return response

    def restart_game_callback(self, request, response):
        """
        Callback for when the game is restarted. This function will reset the clock.

        :param request: The request message
        :param response: The response message
        """

        new_time_base = parse_time_string(request.config.time_base)
        new_time_increment = parse_time_string(request.config.time_increment)
        response.success = self.restart(new_time_base, new_time_increment)
        return response

    def restart(self, new_time_base, new_time_increment):
        self.get_logger().info(
            f"Restarting game with time_base={new_time_base}, time_increment={new_time_increment}"
        )

        if self._clock_connection is None:
            self.get_logger().error("Clock is not connected")
            return False

        cmd = f"rst {new_time_base} {new_time_increment}\n"
        self.get_logger().info(f"Writing {cmd}")
        written = self._clock_connection.write(cmd.encode(encoding="utf-8"))
        if written != len(cmd):
            self.get_logger().error(
                f"Could not write to clock. Wrote {written} bytes of {len(cmd)}"
            )
            return False
        if not self.wait_for_ack():
            self.get_logger().error(f"Clock did not acknowledge command `{cmd}`")
            return False
               
        self._white_time_left = new_time_base
        self._black_time_left = new_time_base
        self._time_pub.publish(
            ChessTime(
                white_time_left=self._white_time_left,
                black_time_left=self._black_time_left,
            )
        )

        self._time_base = new_time_base
        self._time_increment = new_time_increment
        self._game_config_pub.publish(
            GameConfig(time_base=new_time_base, time_increment=new_time_increment)
        )
        return True

    def clock_timer_callback(self):
        """
        Callback for the clock timer. This function will poll the clock and update the game state
        accordingly.
        """

        if self._clock_connection is None:
            return

        msg = self.poll_clock()
        if msg is None:
            self.get_logger().warn("Could not poll clock")
            return

    def handle_message(self, msg):
        """
        Handle a message from the clock.
        """

        m = re.search(
            r"upd w (\d+) b (\d+) t (white|black) p (true|false) a (true|false)", msg.strip()
        )
        if m:
            self._white_time_left = int(m.group(1))
            self._black_time_left = int(m.group(2))
            self._paused = m.group(3) == "true"
            self._active = m.group(4) == "true"
            self._time_pub.publish(
                ChessTime(
                    white_time_left=self._white_time_left,
                    black_time_left=self._black_time_left,
                    paused=self._paused,
                    active=self._active,
                )
            )
            return

        m = re.search(r"btn w (true|false) b (true|false)", msg.strip())
        if m:
            self._white_btn_pressed = m.group(1) == "true"
            self._black_btn_pressed = m.group(2) == "true"
            self._buttons_pub.publish(
                ClockButtons(
                    white_pressed=self._white_btn_pressed,
                    black_pressed=self._black_btn_pressed,
                )
            )
            return


def main(args=None):
    rclpy.init(args=args)

    node = ClockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
