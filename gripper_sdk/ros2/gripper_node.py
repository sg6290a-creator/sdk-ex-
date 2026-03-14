#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from std_srvs.srv import Trigger, SetBool

from gripper_sdk.config.params import BAUDRATE as DEFAULT_BAUDRATE
from gripper_sdk.config.params import MOTOR_IDS as DEFAULT_MOTOR_IDS
from gripper_sdk.config.params import PORT as DEFAULT_PORT
from gripper_sdk.core.gripper import Gripper, MockGripper, OperatingMode
from gripper_sdk.core.motor_group import MotorGroup


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('motor_ids', DEFAULT_MOTOR_IDS)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('use_mock_hardware', True)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        motor_ids = list(self.get_parameter('motor_ids').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        use_mock_hardware = self._as_bool(self.get_parameter('use_mock_hardware').value)

        gripper_cls = MockGripper if use_mock_hardware else Gripper
        self.gripper = gripper_cls(port=port, baudrate=baudrate, motor_ids=motor_ids)
        self.group = MotorGroup(self.gripper)

        self.pub_state = self.create_publisher(Float32MultiArray, 'state', 10)
        self.create_subscription(Int32MultiArray, 'cmd_current', self._cb_cmd_current, 10)
        self.create_subscription(Int32MultiArray, 'cmd_position', self._cb_cmd_position, 10)
        self.create_service(Trigger, 'enable', self._srv_enable)
        self.create_service(Trigger, 'disable', self._srv_disable)
        self.create_service(SetBool, 'set_mode', self._srv_set_mode)
        self.create_timer(max(0.001, 1.0 / publish_rate_hz), self._publish_state)

        hw_mode = 'mock' if use_mock_hardware else 'real'
        self.get_logger().info(f'GripperNode ready ({hw_mode}, port={port}, baudrate={baudrate})')

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _cb_cmd_current(self, msg: Int32MultiArray):
        motor_ids = self.gripper.motor_ids
        if len(msg.data) != len(motor_ids):
            self.get_logger().warn(f'cmd_current: expected {len(motor_ids)} values')
            return
        current_map = {mid: int(msg.data[i]) for i, mid in enumerate(motor_ids)}
        try:
            self.group.write_currents(current_map)
        except Exception as exc:
            self.get_logger().error(f'write_currents failed: {exc}')

    def _cb_cmd_position(self, msg: Int32MultiArray):
        motor_ids = self.gripper.motor_ids
        if len(msg.data) != len(motor_ids):
            self.get_logger().warn(f'cmd_position: expected {len(motor_ids)} values')
            return
        position_map = {mid: int(msg.data[i]) for i, mid in enumerate(motor_ids)}
        try:
            self.group.write_positions(position_map)
        except Exception as exc:
            self.get_logger().error(f'write_positions failed: {exc}')

    def _publish_state(self):
        try:
            motor_ids = self.gripper.motor_ids
            pos = self.group.read_positions()
            vel = self.group.read_velocities()
            cur = self.group.read_currents()

            msg = Float32MultiArray()
            msg.data = (
                [float(pos[mid]) for mid in motor_ids] +
                [float(vel[mid]) for mid in motor_ids] +
                [float(cur[mid]) for mid in motor_ids]
            )
            self.pub_state.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'state read failed: {exc}')

    def _srv_enable(self, _, response):
        try:
            self.gripper.enable()
            response.success = True
            response.message = 'Torque enabled'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _srv_disable(self, _, response):
        try:
            self.gripper.disable()
            response.success = True
            response.message = 'Torque disabled'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _srv_set_mode(self, request, response):
        try:
            self.gripper.disable()
            mode = OperatingMode.CURRENT if request.data else OperatingMode.POSITION
            self.gripper.set_mode(mode)
            self.gripper.enable()
            response.success = True
            response.message = f'Mode set to {mode}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def destroy_node(self):
        try:
            self.gripper.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
