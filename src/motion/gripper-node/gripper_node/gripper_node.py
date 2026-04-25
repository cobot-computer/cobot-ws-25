import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('serial_port', '/dev/ttyGripper')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ack_timeout', 5.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self._is_gripping = False

        try:
            self._serial = serial.Serial(port, baudrate=baud, timeout=0.1)
            time.sleep(2.0)  # let Arduino reset after serial open
            self._serial.reset_input_buffer()  # drain startup message
            self.get_logger().info(f'Gripper connected on {port}')
        except serial.SerialException:
            self.get_logger().error(f'Could not connect to gripper on {port}')
            self._serial = None

        self._srv = self.create_service(SetBool, 'chess/gripper', self._gripper_cb)

    def _send_command(self, cmd: str) -> bool:
        if self._serial is None:
            self.get_logger().error('Gripper serial not connected')
            return False

        timeout = self.get_parameter('ack_timeout').value
        self._serial.reset_input_buffer()
        self._serial.write(f'{cmd}\n'.encode())

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._serial.readline()
            if line:
                if line.decode('utf-8', errors='replace').strip().startswith('ack'):
                    return True
        self.get_logger().error(f'No ack received for gripper command "{cmd}"')
        return False

    def _gripper_cb(self, request, response):
        if request.data:
            # Close: grip piece — Arduino acks immediately, motor keeps running
            self.get_logger().info('Gripper: close')
            success = self._send_command('close')
            if success:
                self._is_gripping = True
            response.success = success
            response.message = 'close' if success else 'close failed — no ack'
        else:
            # Open: release first if currently gripping, then open
            if self._is_gripping:
                self.get_logger().info('Gripper: release')
                if not self._send_command('release'):
                    response.success = False
                    response.message = 'release failed — no ack'
                    return response
                self._is_gripping = False
            self.get_logger().info('Gripper: open')
            success = self._send_command('open')
            response.success = success
            response.message = 'open' if success else 'open failed — no ack'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
