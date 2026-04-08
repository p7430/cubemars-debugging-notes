#!/usr/bin/env python3
"""
High-level Quintic Trajectory Controller (Joint Space)

- Subscribes:
    /joint_states            (sensor_msgs/JointState)  -> current q
    /ik_joint_targets        (sensor_msgs/JointState)  -> goal q

- Publishes:
    joint_position_commands  (std_msgs/Float64MultiArray) -> desired joint positions [q1, q2]
    trajectory_log           (std_msgs/String)            -> human-readable log messages

Behavior:
- When a new IK target arrives, start a new quintic trajectory from current joint state to goal.
- Default duration: 10 seconds
- Safety rule: joint1 delta must be <= 20 degrees per trajectory; otherwise ignore and log.
"""

import math
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def compute_trajectory(position_init, position_goal, t_init, t_goal, t):
    T = t_goal - t_init
    theta_init = position_init
    theta_goal = position_goal

    # Guard against divide-by-zero
    if T <= 1e-9:
        return theta_goal, 0.0

    # Clamp time to [t_init, t_goal]
    if t <= t_init:
        return theta_init, 0.0
    if t >= t_goal:
        return theta_goal, 0.0

    tau = (t - t_init) / T

    # Compute desired position using the 5th-order polynomial
    s_t = (10 * (tau**3) - 15 * (tau**4) + 6 * (tau**5))
    desired_position = theta_init + s_t * (theta_goal - theta_init)

    # Compute desired velocity
    ds_dt = (30 * (tau**2) / T - 60 * (tau**3) / T + 30 * (tau**4) / T)
    desired_velocity = ds_dt * (theta_goal - theta_init)

    return desired_position, desired_velocity


class QuinticTrajectoryController(Node):
    def __init__(self):
        super().__init__("quintic_trajectory_controller")

        # ---- Parameters ----
        self.joint_names: List[str] = self.declare_parameter(
            "joint_names", ["joint1", "joint2"]
        ).value

        self.joint_states_topic: str = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value

        self.targets_topic: str = self.declare_parameter(
            "targets_topic", "/ik_joint_targets"
        ).value

        self.command_topic: str = self.declare_parameter(
            "command_topic", "joint_position_commands"
        ).value

        self.log_topic: str = self.declare_parameter(
            "log_topic", "trajectory_log"
        ).value

        self.duration_s: float = float(self.declare_parameter(
            "trajectory_duration_s", 10.0
        ).value)

        self.publish_rate_hz: float = float(self.declare_parameter(
            "publish_rate_hz", 100.0
        ).value)

        self.max_joint1_delta_deg: float = float(self.declare_parameter(
            "max_joint1_delta_deg", 20.0
        ).value)

        self.max_joint1_delta_rad: float = math.radians(self.max_joint1_delta_deg)

        # ---- State ----
        self._q_current: Optional[List[float]] = None  # [q1, q2]
        self._traj_active: bool = False
        self._t0: float = 0.0
        self._t1: float = 0.0
        self._q0: Optional[List[float]] = None
        self._qg: Optional[List[float]] = None

        # ---- ROS interfaces ----
        self.create_subscription(JointState, self.joint_states_topic, self.on_joint_state, 10)
        self.create_subscription(JointState, self.targets_topic, self.on_target, 10)

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.log_pub = self.create_publisher(String, self.log_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(
            f"QuinticTrajectoryController running.\n"
            f"  joint_states: {self.joint_states_topic}\n"
            f"  targets:      {self.targets_topic}\n"
            f"  commands:     {self.command_topic}\n"
            f"  duration_s:   {self.duration_s}\n"
            f"  rate_hz:      {self.publish_rate_hz}\n"
            f"  max joint1:   {self.max_joint1_delta_deg} deg"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_log(self, text: str):
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)
        self.get_logger().info(text)

    def on_joint_state(self, msg: JointState):
        name_to_idx: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}
        try:
            q1 = float(msg.position[name_to_idx[self.joint_names[0]]])
            q2 = float(msg.position[name_to_idx[self.joint_names[1]]])
        except Exception:
            return

        self._q_current = [q1, q2]

    def on_target(self, msg: JointState):
        if self._q_current is None:
            self.publish_log("Trajectory ignored: no /joint_states received yet.")
            return

        name_to_idx: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}
        try:
            qg1 = float(msg.position[name_to_idx[self.joint_names[0]]])
            qg2 = float(msg.position[name_to_idx[self.joint_names[1]]])
        except Exception:
            self.publish_log("Trajectory ignored: target JointState missing required joint names.")
            return

        q0 = self._q_current[:]  # start from current measured state
        qg = [qg1, qg2]

        # Safety rule: joint1 delta limit
        d1 = qg[0] - q0[0]
        if abs(d1) > self.max_joint1_delta_rad:
            self._traj_active = False
            self._q0 = None
            self._qg = None
            self.publish_log(
                f"Didn't move: joint1 command exceeds limit. "
                f"Δq1={math.degrees(d1):.2f} deg > {self.max_joint1_delta_deg:.2f} deg. "
                f"(q1: {q0[0]:.4f} -> {qg[0]:.4f} rad)"
            )
            return

        # Start trajectory
        t0 = self.now_s()
        t1 = t0 + max(1e-3, self.duration_s)

        self._q0 = q0
        self._qg = qg
        self._t0 = t0
        self._t1 = t1
        self._traj_active = True

        self.publish_log(
            f"Trajectory started: {self.duration_s:.2f}s "
            f"q0=[{q0[0]:.4f}, {q0[1]:.4f}] -> qg=[{qg[0]:.4f}, {qg[1]:.4f}] (rad)"
        )

    def on_timer(self):
        # Only publish if we have a trajectory active and a current state
        if not self._traj_active or self._q0 is None or self._qg is None:
            return

        t = self.now_s()

        q_cmd = [0.0, 0.0]
        # Compute quintic position for each joint
        for i in range(2):
            pos, _vel = compute_trajectory(self._q0[i], self._qg[i], self._t0, self._t1, t)
            q_cmd[i] = float(pos)

        # Publish command to motor driver
        out = Float64MultiArray()
        out.data = q_cmd
        self.cmd_pub.publish(out)

        # End condition
        if t >= self._t1:
            self._traj_active = False
            self.publish_log(
                f"Trajectory complete: final command [{q_cmd[0]:.4f}, {q_cmd[1]:.4f}] (rad)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = QuinticTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

