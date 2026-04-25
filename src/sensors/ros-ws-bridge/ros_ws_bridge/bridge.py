import asyncio
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from chess_msgs.msg import FullFEN, ChessTime, ClockButtons
import websockets

CLOCK_BROADCAST_INTERVAL = 1.0  # seconds between clock updates to GUI


class RosWsBridge(Node):
    def __init__(self):
        super().__init__('ros_ws_bridge')
        self._ws_clients: set = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._last_turn = 'white'
        self._last_clock_broadcast = 0.0
        self._last_clock_msg: str | None = None
        self._last_fen_msg: str | None = None

        self.create_subscription(FullFEN, 'chess/game_state', self._game_state_cb, 10)
        self.create_subscription(ChessTime, 'chess/time', self._time_cb, 10)
        self.create_subscription(ClockButtons, 'chess/clock_buttons', self._clock_buttons_cb, 10)

    def _game_state_cb(self, msg: FullFEN):
        parts = msg.fen.split(' ')
        if len(parts) >= 2:
            self._last_turn = 'white' if parts[1] == 'w' else 'black'
        self._last_fen_msg = msg.fen.strip()
        self._broadcast(self._last_fen_msg)

    def _clock_buttons_cb(self, msg: ClockButtons):
        # White pressed their button → their turn just ended → now black's turn, and vice versa.
        if msg.white_pressed:
            self._last_turn = 'black'
        elif msg.black_pressed:
            self._last_turn = 'white'
        # Force an immediate clock broadcast so the GUI sees the turn change at once.
        if self._last_clock_msg and self._ws_clients:
            parts = self._last_clock_msg.split(':')
            if len(parts) == 5:
                updated = f'clock:{parts[1]}:{parts[2]}:{self._last_turn}:{parts[4]}'
                self._last_clock_msg = updated
                self._broadcast(updated)

    def _time_cb(self, msg: ChessTime):
        now = time.monotonic()
        self._last_turn = 'white' if msg.white_to_play else 'black'
        if now - self._last_clock_broadcast < CLOCK_BROADCAST_INTERVAL:
            return
        self._last_clock_broadcast = now
        running = 'true' if not msg.paused else 'false'
        self._last_clock_msg = (
            f'clock:{msg.white_time_left}:{msg.black_time_left}:{self._last_turn}:{running}'
        )
        if self._ws_clients:
            self._broadcast(self._last_clock_msg)

    def _broadcast(self, message: str):
        if self._loop is not None and self._ws_clients:
            asyncio.run_coroutine_threadsafe(self._async_broadcast(message), self._loop)

    async def _async_broadcast(self, message: str):
        for client in self._ws_clients.copy():
            try:
                await client.send(message)
            except Exception:
                self._ws_clients.discard(client)

    async def _ws_handler(self, websocket):
        self._ws_clients.add(websocket)
        self.get_logger().info(f'Client connected: {websocket.remote_address}')
        # Replay the last board position so a reconnecting client sees the right pieces.
        # Clock is intentionally NOT replayed — the live 1 Hz broadcast arrives within
        # one second and avoids flashing stale values from a previous session.
        try:
            if self._last_fen_msg:
                await websocket.send(self._last_fen_msg)
        except Exception:
            pass
        try:
            async for message in websocket:
                self.get_logger().info(f'Received from GUI: {message}')
        except Exception:
            pass
        finally:
            self._ws_clients.discard(websocket)
            self.get_logger().info('Client disconnected')

    async def _serve(self):
        self._loop = asyncio.get_running_loop()

        # Cancel all tasks on SIGTERM so the process exits cleanly and releases
        # port 8080 — without this the bridge becomes a zombie that blocks the
        # next launch from binding the port.
        def _stop():
            for task in asyncio.all_tasks(self._loop):
                task.cancel()
        self._loop.add_signal_handler(signal.SIGTERM, _stop)
        self._loop.add_signal_handler(signal.SIGINT, _stop)

        self.get_logger().info('WebSocket server started on ws://localhost:8080')
        try:
            async with websockets.serve(self._ws_handler, 'localhost', 8080):
                await asyncio.Future()
        except asyncio.CancelledError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RosWsBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(node._serve())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
