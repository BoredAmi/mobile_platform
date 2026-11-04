"""mcu_temperature: the MCP9808 on the MCU board (RP2040 firmware 1.1+) as sensor_msgs/Temperature.

The board reads the sensor on its I2C bus and sends it over USB; jgb_rover_hardware exposes it as the
state interface temp_sensor/temperature, which joint_state_broadcaster publishes on
/dynamic_joint_states. This node republishes it like mcp9808_driver (the Pi-I2C variant) does, so
temperature_mapper works with either. NaN (sensor missing / stale) is not published.
"""
import math

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class McuTemperature(Node):
    def __init__(self):
        super().__init__('mcu_temperature')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.topic = p('topic', '/temperature')
        self.frame_id = p('frame_id', 'temp_sensor_link')
        self.sensor_name = p('sensor_name', 'temp_sensor')
        rate = float(p('rate_hz', 2.0))
        self.variance = float(p('accuracy_c', 0.25)) ** 2
        self.offset = float(p('offset_c', 0.0))          # calibration: added to every reading

        self.value = math.nan
        self.warned = False
        self.pub = self.create_publisher(Temperature, self.topic, 10)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.on_state, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'{self.sensor_name}/temperature from /dynamic_joint_states -> {self.topic}')

    def on_state(self, msg: DynamicJointState):
        if self.sensor_name not in msg.joint_names:
            return
        values = msg.interface_values[msg.joint_names.index(self.sensor_name)]
        if 'temperature' in values.interface_names:
            self.value = values.values[values.interface_names.index('temperature')]

    def tick(self):
        if not math.isfinite(self.value):
            if not self.warned:
                self.get_logger().warn('no temperature from the MCU board yet (MCP9808 missing, or '
                                       'firmware older than 1.1?)', throttle_duration_sec=10.0)
            return
        self.warned = False
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = self.value + self.offset
        msg.variance = self.variance
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = McuTemperature()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
