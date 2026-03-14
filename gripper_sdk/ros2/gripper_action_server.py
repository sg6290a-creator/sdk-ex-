#!/usr/bin/env python3
"""
GripperCommand action server for MoveIt2 integration.

Maps a single GripperCommand position (0.0=open, 1.0=closed) to
the multi-motor gripper via the existing gripper_node topics.
"""
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')

        # --- parameters -----------------------------------------------------------
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        self.declare_parameter('open_positions',  [0]*11)      # per-motor open
        self.declare_parameter('closed_positions', [4095]*11)   # per-motor closed
        self.declare_parameter('position_tolerance', 50.0)      # encoder ticks
        self.declare_parameter('goal_timeout_sec', 5.0)
        self.declare_parameter('state_timeout_sec', 1.0)
        self.declare_parameter('feedback_rate_hz', 20.0)

        self._motor_ids       = list(self.get_parameter('motor_ids').value)
        self._open_pos        = list(self.get_parameter('open_positions').value)
        self._closed_pos      = list(self.get_parameter('closed_positions').value)
        self._tolerance       = float(self.get_parameter('position_tolerance').value)
        self._goal_timeout    = float(self.get_parameter('goal_timeout_sec').value)
        self._state_timeout   = float(self.get_parameter('state_timeout_sec').value)
        self._feedback_rate   = float(self.get_parameter('feedback_rate_hz').value)

        n = len(self._motor_ids)
        assert len(self._open_pos) == n and len(self._closed_pos) == n, \
            'open_positions / closed_positions length must match motor_ids'

        # --- comms with gripper_node -----------------------------------------------
        cb = ReentrantCallbackGroup()
        self._pub_cmd = self.create_publisher(Int32MultiArray, 'cmd_position', 10)
        self._last_state: Float32MultiArray | None = None
        self._state_stamp = self.get_clock().now()
        self.create_subscription(
            Float32MultiArray, 'state', self._on_state, 10, callback_group=cb)

        # --- action server ---------------------------------------------------------
        self._action_server = ActionServer(
            self, GripperCommand, 'gripper_cmd',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb,
        )
        self.get_logger().info('GripperActionServer ready')

    # ---- callbacks ----------------------------------------------------------------

    def _goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _on_state(self, msg: Float32MultiArray):
        self._last_state = msg
        self._state_stamp = self.get_clock().now()

    # ---- helpers ------------------------------------------------------------------

    def _interpolate(self, ratio: float) -> list[int]:
        """Map normalised position [0,1] → per-motor encoder targets."""
        ratio = max(0.0, min(1.0, ratio))
        return [
            int(o + ratio * (c - o))
            for o, c in zip(self._open_pos, self._closed_pos)
        ]

    def _current_positions(self) -> list[float] | None:
        """Extract per-motor positions from the latest state message."""
        if self._last_state is None:
            return None
        n = len(self._motor_ids)
        if len(self._last_state.data) < n:
            return None
        return [float(self._last_state.data[i]) for i in range(n)]

    def _current_efforts(self) -> list[float]:
        """Extract per-motor currents (used as effort proxy)."""
        if self._last_state is None:
            return [0.0] * len(self._motor_ids)
        n = len(self._motor_ids)
        # state layout: [pos*n, vel*n, cur*n]
        offset = 2 * n
        if len(self._last_state.data) < offset + n:
            return [0.0] * n
        return [float(self._last_state.data[offset + i]) for i in range(n)]

    def _position_to_ratio(self, positions: list[float]) -> float:
        """Compute mean normalised position from encoder values."""
        ratios = []
        for pos, o, c in zip(positions, self._open_pos, self._closed_pos):
            span = c - o
            ratios.append((pos - o) / span if span != 0 else 0.0)
        return sum(ratios) / len(ratios) if ratios else 0.0

    def _mean_effort(self) -> float:
        efforts = self._current_efforts()
        return sum(abs(e) for e in efforts) / len(efforts) if efforts else 0.0

    # ---- execute ------------------------------------------------------------------

    async def _execute_cb(self, goal_handle):
        goal: GripperCommand.Goal = goal_handle.request
        target_ratio = goal.command.position          # 0.0 = open, 1.0 = closed
        max_effort   = goal.command.max_effort         # 0 ⇒ no limit

        targets = self._interpolate(target_ratio)
        self.get_logger().info(
            f'Goal: position={target_ratio:.3f}, max_effort={max_effort:.1f}')

        # send command
        cmd = Int32MultiArray(data=targets)
        self._pub_cmd.publish(cmd)

        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()
        rate = self.create_rate(self._feedback_rate)
        start = self.get_clock().now()

        while rclpy.ok():
            # cancelled?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached_goal = False
                result.position = self._position_to_ratio(
                    self._current_positions() or self._open_pos)
                result.effort = self._mean_effort()
                return result

            # state timeout
            age = (self.get_clock().now() - self._state_stamp).nanoseconds * 1e-9
            if age > self._state_timeout:
                self.get_logger().warn('State timeout – no recent gripper state')

            cur_pos = self._current_positions()
            if cur_pos is not None:
                ratio_now = self._position_to_ratio(cur_pos)
                effort_now = self._mean_effort()

                # stall detection: effort exceeds limit
                stalled = max_effort > 0.0 and effort_now > max_effort

                # convergence check
                errors = [abs(p - t) for p, t in zip(cur_pos, targets)]
                reached = all(e <= self._tolerance for e in errors)

                # feedback
                feedback.position = ratio_now
                feedback.effort = effort_now
                feedback.stalled = stalled
                feedback.reached_goal = reached
                goal_handle.publish_feedback(feedback)

                if reached or stalled:
                    result.position = ratio_now
                    result.effort = effort_now
                    result.stalled = stalled
                    result.reached_goal = reached
                    goal_handle.succeed()
                    self.get_logger().info(
                        f'Done: reached={reached}, stalled={stalled}, '
                        f'pos={ratio_now:.3f}')
                    return result

            # goal timeout
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed > self._goal_timeout:
                self.get_logger().warn('Goal timed out')
                result.position = self._position_to_ratio(
                    self._current_positions() or self._open_pos)
                result.effort = self._mean_effort()
                result.reached_goal = False
                result.stalled = False
                goal_handle.abort()
                return result

            rate.sleep()

        # node shutting down
        result.reached_goal = False
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
