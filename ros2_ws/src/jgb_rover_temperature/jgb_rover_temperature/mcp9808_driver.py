"""mcp9808_driver: read an MCP9808 over Linux I2C (e.g. Raspberry Pi I2C1) and publish it.

Publishes: topic (sensor_msgs/Temperature, frame_id = frame_id, variance = accuracy_c^2)
Needs python3-smbus2 (apt) or python3-smbus. sim_mcp9808 publishes the same message in simulation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

from jgb_rover_temperature.mcp9808 import MCP9808


def open_bus(bus_id: int):
    try:
        from smbus2 import SMBus
    except ImportError:
        from smbus import SMBus
    return SMBus(bus_id)


class Mcp9808Driver(Node):
    def __init__(self):
        super().__init__('mcp9808_driver')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.topic = p('topic', '/temperature')
        self.frame_id = p('frame_id', 'temp_sensor_link')
        bus_id = int(p('i2c_bus', 1.0))
        address = int(p('i2c_address', 24.0))
        resolution = float(p('resolution_c', 0.0625))
        rate = float(p('rate_hz', 2.0))
        self.variance = float(p('accuracy_c', 0.25)) ** 2
        self.offset = float(p('offset_c', 0.0))          # calibration: added to every reading

        self.sensor = MCP9808(open_bus(bus_id), address)
        self.sensor.check_ids()
        self.sensor.configure(resolution)
        self.errors = 0
        self.pub = self.create_publisher(Temperature, self.topic, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'MCP9808 on i2c-{bus_id} 0x{address:02x}, {resolution} C, {rate} Hz -> {self.topic}')

    def tick(self):
        try:
            t = self.sensor.read_celsius()
        except OSError as e:
            self.errors += 1
            self.get_logger().warn(f'I2C read failed ({self.errors} so far): {e}', throttle_duration_sec=5.0)
            return
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = t + self.offset
        msg.variance = self.variance
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mcp9808Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
