#!/usr/bin/env python3
"""
MIT Mode Motor Driver for ROS2 (SAFE startup + explicit calibrate/arm)

Key behavior changes vs your original:
- On startup: EXIT -> ENTER, then SAFE mode (kp=0) while syncing feedback.
- No automatic ZERO on boot (prevents reference-frame jumps).
- Commands on /joint_position_commands are ignored unless ARMED.
- Services:
   /calibrate   (std_srvs/Trigger) : performs MIT_ZERO safely + resync setpoints
   /arm         (std_srvs/SetBool) : True=arm (ramps gains), False=disarm (kp->0)
- On shutdown (SIGINT/SIGTERM): sends a short benign command burst then MIT_EXIT.

IMPORTANT FIX:
- Do NOT install a custom SIGINT handler (it can swallow Ctrl+C).
- Install SIGTERM handler only, and have it call rclpy.shutdown() so ros2 launch stop works.

COMPATIBILITY UPDATE (keeps old behavior):
- The upstream trajectory node can publish:
    [q1, q2]                      (old)
    [q1, q2, qd1, qd2]            (new)
- This driver now supports both, controlled by parameter:
    accept_velocity_in_command (default: True)
  If True:
    - len==N  -> positions only, velocities set to 0.0
    - len==2N -> positions + velocities
  If False:
    - requires len==N (old strict mode)
"""


import rclpy
from rclpy.node import Node


from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool


import can
import time
import math
import signal
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class MITRanges:
   """Motor-specific parameter ranges from manual"""
   v_min: float
   v_max: float
   t_min: float
   t_max: float


MOTOR_CONFIGS = {
   'AK60-6':  MITRanges(v_min=-45.0, v_max=45.0, t_min=-15.0, t_max=15.0),
   'AK70-10': MITRanges(v_min=-50.0, v_max=50.0, t_min=-25.0, t_max=25.0),
   'AK80-6':  MITRanges(v_min=-76.0, v_max=76.0, t_min=-12.0, t_max=12.0),
   'AK80-9':  MITRanges(v_min=-50.0, v_max=50.0, t_min=-18.0, t_max=18.0),
}


# MIT mode constants
P_MIN, P_MAX = -12.5, 12.5  # Position range in radians
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0


# Special MIT commands
MIT_ENTER = bytes([0xFF] * 7 + [0xFC])
MIT_EXIT  = bytes([0xFF] * 7 + [0xFD])
MIT_ZERO  = bytes([0xFF] * 7 + [0xFE])


class MITMotorDriver(Node):
   """
   ROS2 node for MIT mode motor control.
   Default behavior is SAFE (kp=0) until you explicitly /arm.

   Topics:
     Publishes: joint_states (sensor_msgs/JointState)
     Subscribes: joint_position_commands (std_msgs/Float64MultiArray)  [ignored unless ARMED]

   Services:
     /calibrate (std_srvs/Trigger)  -> MIT_ZERO safely + resync setpoints
     /arm       (std_srvs/SetBool)  -> True=ARM (ramp gains), False=DISARM
   """

   def __init__(self):
       super().__init__('mit_motor_driver')

       # ------------------------
       # Parameters
       # ------------------------
       self.declare_parameter('can_interface', 'can0')
       self.declare_parameter('control_rate_hz', 100.0)
       self.declare_parameter('publish_rate_hz', 50.0)

       self.declare_parameter('motor_ids', [1, 2])
       self.declare_parameter('motor_types', ['AK60-6', 'AK70-10'])
       self.declare_parameter('joint_names', ['joint1', 'joint2'])

       # "armed" gains (what you want during normal operation)
       self.declare_parameter('default_kp', 24.0)
       self.declare_parameter('default_kd', 2.5)

       # SAFE mode behavior
       self.declare_parameter('safe_kp', 0.0)        # usually 0.0
       self.declare_parameter('safe_kd', 0.5)        # small damping helps reduce floppiness
       self.declare_parameter('startup_sync_timeout_s', 2.0)
       self.declare_parameter('calib_settle_time_s', 0.15)
       self.declare_parameter('ramp_time_s', 2.0)    # seconds to ramp gains when arming
       self.declare_parameter('arm_requires_calibrate', False)  # if True, blocks arm until calibrate done

       # How to identify motor from replies
       self.declare_parameter('reply_id_source', 'payload')  # 'payload' or 'arbitration'
       # If payload: expects rx.data[0] == motor_id. If arbitration: rx.arbitration_id == motor_id.

       # NEW: accept [q..., qd...] command format from trajectory node (backward compatible)
       self.declare_parameter('accept_velocity_in_command', True)

       # ------------------------
       # Load params
       # ------------------------
       self.can_interface = self.get_parameter('can_interface').value
       self.control_rate = float(self.get_parameter('control_rate_hz').value)
       self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

       self.motor_ids = list(self.get_parameter('motor_ids').value)
       motor_types = list(self.get_parameter('motor_types').value)
       self.joint_names = list(self.get_parameter('joint_names').value)

       self.default_kp = float(self.get_parameter('default_kp').value)
       self.default_kd = float(self.get_parameter('default_kd').value)

       self.safe_kp = float(self.get_parameter('safe_kp').value)
       self.safe_kd = float(self.get_parameter('safe_kd').value)
       self.startup_sync_timeout_s = float(self.get_parameter('startup_sync_timeout_s').value)
       self.calib_settle_time_s = float(self.get_parameter('calib_settle_time_s').value)
       self.ramp_time_s = float(self.get_parameter('ramp_time_s').value)
       self.arm_requires_calibrate = bool(self.get_parameter('arm_requires_calibrate').value)

       self.reply_id_source = str(self.get_parameter('reply_id_source').value).strip().lower()
       if self.reply_id_source not in ('payload', 'arbitration'):
           raise ValueError("reply_id_source must be 'payload' or 'arbitration'")

       self.accept_velocity_in_command = bool(self.get_parameter('accept_velocity_in_command').value)

       if not (len(self.motor_ids) == len(motor_types) == len(self.joint_names)):
           raise ValueError('motor_ids, motor_types, and joint_names must have same length')

       # Setup motor ranges
       self.motor_ranges: Dict[int, MITRanges] = {}
       for motor_id, motor_type in zip(self.motor_ids, motor_types):
           if motor_type not in MOTOR_CONFIGS:
               raise ValueError(f'Unknown motor type: {motor_type}')
           self.motor_ranges[motor_id] = MOTOR_CONFIGS[motor_type]

       # ------------------------
       # CAN bus
       # ------------------------
       try:
           self.bus = can.interface.Bus(interface='socketcan', channel=self.can_interface)
           self.get_logger().info(f'Opened CAN interface {self.can_interface}')
       except Exception as e:
           self.get_logger().error(f'Failed to open CAN: {e}')
           raise

       # ------------------------
       # State
       # ------------------------
       self.motor_states = {
           motor_id: {
               'position': 0.0,
               'velocity': 0.0,
               'effort': 0.0,
               'temperature': 0,
               'error_flag': 0,
               'last_update': 0.0  # start as "not updated"
           }
           for motor_id in self.motor_ids
       }

       # Desired state (commands)
       self.desired_positions = {motor_id: 0.0 for motor_id in self.motor_ids}
       self.desired_velocities = {motor_id: 0.0 for motor_id in self.motor_ids}
       self.desired_torque_ff = {motor_id: 0.0 for motor_id in self.motor_ids}

       # Gains are stateful: SAFE vs ARMED vs ramping
       self.desired_kp = {motor_id: self.safe_kp for motor_id in self.motor_ids}
       self.desired_kd = {motor_id: self.safe_kd for motor_id in self.motor_ids}

       self._last_stale_warn = {}
       self._last_error_warn = {}

       self._armed = False
       self._calibrated_once = False

       self._ramping = False
       self._ramp_start_t = 0.0
       self._ramp_from_kp = 0.0
       self._ramp_from_kd = 0.0
       self._ramp_to_kp = 0.0
       self._ramp_to_kd = 0.0

       self._shutdown_requested = False

       # ------------------------
       # ROS interfaces
       # ------------------------
       self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

       self.cmd_sub = self.create_subscription(
           Float64MultiArray,
           'joint_position_commands',
           self.command_callback,
           10
       )

       self.calib_srv = self.create_service(Trigger, 'calibrate', self.handle_calibrate)
       self.arm_srv = self.create_service(SetBool, 'arm', self.handle_arm)

       # Timers
       self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
       self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

       # Signal handlers (best effort)
       # IMPORTANT: do NOT override SIGINT (Ctrl+C). That can swallow KeyboardInterrupt.
       # We only handle SIGTERM so ros2 launch stop will exit cleanly.
       try:
           signal.signal(signal.SIGTERM, self._signal_handler)
       except Exception:
           pass

       # ------------------------
       # Startup sequence: clean mode, SAFE, sync
       # ------------------------
       self.startup_sequence()

       self.get_logger().info(
           'MITMotorDriver ready. Default = SAFE (kp=0). '
           'Use /calibrate then /arm true to enable stiffness.'
       )
       self.get_logger().info(
           f'Command format support: accept_velocity_in_command={self.accept_velocity_in_command} '
           f'(accepts [q..] and optionally [q..,qd..]).'
       )

   # ------------------------
   # Safety + lifecycle
   # ------------------------
   def _signal_handler(self, signum, frame):
       # Request shutdown and force rclpy.spin() to exit (SIGTERM path).
       self._shutdown_requested = True
       try:
           rclpy.shutdown()
       except Exception:
           pass

   def startup_sequence(self):
       self.get_logger().info('Startup: forcing clean MIT state (EXIT->ENTER), SAFE gains, syncing feedback...')
       for motor_id in self.motor_ids:
           try:
               # Clean slate: exit then enter
               self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=MIT_EXIT))
               time.sleep(0.02)
               self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=MIT_ENTER))
               time.sleep(0.02)
           except can.CanError as e:
               self.get_logger().error(f'CAN error during startup for motor {motor_id}: {e}')
               raise

       # Flush old messages
       self._flush_bus()

       # SAFE command burst while we gather feedback
       self._set_safe_mode()
       self._safe_burst(duration_s=0.25)

       # Sync: wait for fresh feedback then hold that position (still SAFE kp=0)
       ok = self._sync_setpoints(timeout_s=self.startup_sync_timeout_s)
       if not ok:
           self.get_logger().warn(
               'Startup sync timed out: did not receive fresh feedback from all motors. '
               'Staying SAFE (kp=0). Check wiring/CAN bitrate/reply_id_source.'
           )
       else:
           self.get_logger().info('Startup sync complete: setpoints aligned to measured positions (SAFE).')

   def _flush_bus(self):
       while self.bus.recv(timeout=0.001) is not None:
           pass

   def _set_safe_mode(self):
       self._armed = False
       self._ramping = False
       for mid in self.motor_ids:
           self.desired_kp[mid] = self.safe_kp
           self.desired_kd[mid] = self.safe_kd

   def _safe_burst(self, duration_s: float):
       """
       Send SAFE frames (kp=0) for duration_s while draining RX.
       Useful to overwrite stale internal setpoints / stabilize comms.
       """
       t_end = time.time() + max(0.0, duration_s)
       while time.time() < t_end:
           # Send SAFE commands
           for motor_id in self.motor_ids:
               try:
                   cmd = self.pack_mit_cmd(
                       p_des=self.desired_positions[motor_id],  # doesn't matter much when kp=0
                       v_des=0.0,
                       kp=self.safe_kp,
                       kd=self.safe_kd,
                       t_ff=0.0,
                       ranges=self.motor_ranges[motor_id]
                   )
                   self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=cmd))
               except Exception:
                   pass
           # Drain RX briefly
           self._drain_rx(window_s=0.002)
           time.sleep(0.005)

   def _sync_setpoints(self, timeout_s: float) -> bool:
       """
       Wait until each motor has a fresh state update, then set desired_positions = measured position.
       Returns True if all motors updated within timeout.
       """
       t0 = time.time()
       updated = {mid: False for mid in self.motor_ids}

       while time.time() - t0 < max(0.0, timeout_s):
           self._drain_rx(window_s=0.005)
           now = time.time()
           for mid in self.motor_ids:
               if self.motor_states[mid]['last_update'] > 0.0 and (now - self.motor_states[mid]['last_update'] < 0.2):
                   updated[mid] = True
           if all(updated.values()):
               # Align setpoints to measured positions
               for mid in self.motor_ids:
                   self.desired_positions[mid] = float(self.motor_states[mid]['position'])
                   self.desired_velocities[mid] = 0.0
                   self.desired_torque_ff[mid] = 0.0
               return True

           # keep SAFE frames coming during sync
           self._safe_burst(duration_s=0.02)

       return False

   # ------------------------
   # Services
   # ------------------------
   def handle_calibrate(self, request: Trigger.Request, response: Trigger.Response):
       """
       Calibrate = safely issue MIT_ZERO, then resync setpoints.
       """
       self.get_logger().warn('CALIBRATE requested: entering SAFE, issuing MIT_ZERO, resyncing...')
       self._set_safe_mode()

       # brief SAFE burst before zero
       self._safe_burst(duration_s=0.15)

       # send ZERO per motor
       for motor_id in self.motor_ids:
           try:
               self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=MIT_ZERO))
               time.sleep(0.03)
           except can.CanError as e:
               msg = f'CAN error sending MIT_ZERO to motor {motor_id}: {e}'
               self.get_logger().error(msg)
               response.success = False
               response.message = msg
               return response

       # let it settle, then flush and resync
       time.sleep(max(0.0, self.calib_settle_time_s))
       self._flush_bus()
       self._safe_burst(duration_s=0.2)

       ok = self._sync_setpoints(timeout_s=1.5)
       if not ok:
           response.success = False
           response.message = 'Calibrate: did not receive fresh feedback from all motors after zero. Staying SAFE.'
           self.get_logger().error(response.message)
           return response

       self._calibrated_once = True
       response.success = True
       response.message = 'Calibrate complete: MIT_ZERO sent, setpoints synced (SAFE, not armed).'
       self.get_logger().info(response.message)
       return response

   def handle_arm(self, request: SetBool.Request, response: SetBool.Response):
       """
       Arm True: ramp gains from SAFE -> default over ramp_time_s, keep setpoint = current.
       Arm False: disarm immediately (kp=0).
       """
       want_arm = bool(request.data)

       if not want_arm:
           self.get_logger().warn('DISARM requested: kp->0 (SAFE).')
           self._set_safe_mode()
           response.success = True
           response.message = 'Disarmed: SAFE gains applied (kp=0).'
           return response

       # want arm == True
       if self.arm_requires_calibrate and not self._calibrated_once:
           response.success = False
           response.message = 'Arm blocked: arm_requires_calibrate=True and no successful /calibrate yet.'
           self.get_logger().warn(response.message)
           return response

       # sync setpoints to current position before ramping
       ok = self._sync_setpoints(timeout_s=0.8)
       if not ok:
           response.success = False
           response.message = 'Arm failed: could not sync to fresh motor feedback. Staying SAFE.'
           self.get_logger().error(response.message)
           return response

       self.get_logger().warn(f'ARM requested: ramping gains over {self.ramp_time_s:.2f}s.')
       self._armed = True
       self._ramping = True
       self._ramp_start_t = time.time()

       # ramp from current gains (likely SAFE) to defaults
       # (use motor0 to define; we apply same kp/kd to all motors)
       any_mid = self.motor_ids[0]
       self._ramp_from_kp = float(self.desired_kp[any_mid])
       self._ramp_from_kd = float(self.desired_kd[any_mid])
       self._ramp_to_kp = float(self.default_kp)
       self._ramp_to_kd = float(self.default_kd)

       response.success = True
       response.message = 'Arming started: ramping gains. Commands will be accepted.'
       return response

   # ------------------------
   # ROS callbacks
   # ------------------------
   def command_callback(self, msg: Float64MultiArray):
       """
       Update desired positions (and optionally desired velocities) from command topic.
       Ignored unless armed.

       Supported formats (N = number of motors):
         - [q1..qN]                    (old)
         - [q1..qN, qd1..qdN]          (new)
       """
       if not self._armed:
           # ignore commands while SAFE
           return

       n = len(self.motor_ids)
       m = len(msg.data)

       if not self.accept_velocity_in_command:
           # strict old mode
           if m != n:
               self.get_logger().warn(f'Command length {m} != motor count {n} (strict mode)')
               return
           for i, motor_id in enumerate(self.motor_ids):
               self.desired_positions[motor_id] = float(msg.data[i])
               self.desired_velocities[motor_id] = 0.0
           return

       # flexible mode: accept N or 2N
       if m == n:
           for i, motor_id in enumerate(self.motor_ids):
               self.desired_positions[motor_id] = float(msg.data[i])
               self.desired_velocities[motor_id] = 0.0  # no vel provided
           return

       if m == 2 * n:
           for i, motor_id in enumerate(self.motor_ids):
               self.desired_positions[motor_id] = float(msg.data[i])
               self.desired_velocities[motor_id] = float(msg.data[n + i])
           return

       self.get_logger().warn(
           f'Command length {m} not supported. Expected {n} ([q..]) or {2*n} ([q..,qd..]).'
       )

   # ------------------------
   # Control loop
   # ------------------------
   def control_loop(self):
       if self._shutdown_requested:
           # let main finally() handle shutdown sequence
           return

       # Apply ramp if arming
       if self._ramping:
           self._update_ramp_gains()

       # Send commands
       for motor_id in self.motor_ids:
           try:
               cmd = self.pack_mit_cmd(
                   p_des=self.desired_positions[motor_id],
                   v_des=self.desired_velocities[motor_id],
                   kp=self.desired_kp[motor_id],
                   kd=self.desired_kd[motor_id],
                   t_ff=self.desired_torque_ff[motor_id],
                   ranges=self.motor_ranges[motor_id]
               )
               self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=cmd))
           except can.CanError:
               self.get_logger().error(f'CAN send failed for motor {motor_id}')
           except Exception as e:
               self.get_logger().error(f'Error packing/sending for motor {motor_id}: {e}')

       # Drain RX briefly
       self._drain_rx(window_s=0.002)

   def _update_ramp_gains(self):
       T = max(1e-3, float(self.ramp_time_s))
       t = time.time() - self._ramp_start_t
       alpha = min(1.0, max(0.0, t / T))

       # smoothstep for gentler start/stop
       s = alpha * alpha * (3.0 - 2.0 * alpha)

       kp_now = (1.0 - s) * self._ramp_from_kp + s * self._ramp_to_kp
       kd_now = (1.0 - s) * self._ramp_from_kd + s * self._ramp_to_kd

       for mid in self.motor_ids:
           self.desired_kp[mid] = float(kp_now)
           self.desired_kd[mid] = float(kd_now)

       if alpha >= 1.0:
           self._ramping = False
           self.get_logger().info(f'Arming complete: kp={self._ramp_to_kp:.2f}, kd={self._ramp_to_kd:.2f}.')

   def _drain_rx(self, window_s: float):
       t_end = time.time() + max(0.0, window_s)
       while time.time() < t_end:
           rx = self.bus.recv(timeout=0.0)
           if rx is None:
               break
           if rx.dlc != 8 or rx.is_extended_id:
               continue

           if self.reply_id_source == 'payload':
               motor_id = rx.data[0]
           else:
               motor_id = rx.arbitration_id

           if motor_id not in self.motor_ranges:
               continue

           state = self.unpack_mit_reply(bytes(rx.data), self.motor_ranges[motor_id])
           if state:
               self.motor_states[motor_id] = state

   # ------------------------
   # Publishing
   # ------------------------
   def publish_joint_states(self):
       msg = JointState()
       msg.header.stamp = self.get_clock().now().to_msg()
       msg.name = self.joint_names
       msg.position = []
       msg.velocity = []
       msg.effort = []

       now = time.time()
       for motor_id in self.motor_ids:
           state = self.motor_states[motor_id]
           msg.position.append(state['position'])
           msg.velocity.append(state['velocity'])
           msg.effort.append(state['effort'])

           # stale data warning (manual throttle)
           if state['last_update'] <= 0.0 or (now - state['last_update'] > 0.1):
               last = self._last_stale_warn.get(motor_id, 0.0)
               if now - last > 1.0:
                   self.get_logger().warn(f'Motor {motor_id} state stale (>100ms)')
                   self._last_stale_warn[motor_id] = now

           # error flag warning (manual throttle)
           if state['error_flag'] != 0:
               last = self._last_error_warn.get(motor_id, 0.0)
               if now - last > 1.0:
                   self.get_logger().error(f'Motor {motor_id} error flag: {state["error_flag"]}')
                   self._last_error_warn[motor_id] = now

       self.joint_state_pub.publish(msg)

   # ------------------------
   # MIT packing helpers
   # ------------------------
   @staticmethod
   def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
       span = x_max - x_min
       x = max(x_min, min(x_max, x))
       return int((x - x_min) * ((1 << bits) - 1) / span)

   @staticmethod
   def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
       span = x_max - x_min
       return float(x) * span / float((1 << bits) - 1) + x_min

   def pack_mit_cmd(self, p_des: float, v_des: float, kp: float,
                    kd: float, t_ff: float, ranges: MITRanges) -> bytes:
       # Clamp values
       p_des = max(P_MIN, min(P_MAX, p_des))
       v_des = max(ranges.v_min, min(ranges.v_max, v_des))
       kp = max(KP_MIN, min(KP_MAX, kp))
       kd = max(KD_MIN, min(KD_MAX, kd))
       t_ff = max(ranges.t_min, min(ranges.t_max, t_ff))

       # Convert to uints
       p_int = self.float_to_uint(p_des, P_MIN, P_MAX, 16)
       v_int = self.float_to_uint(v_des, ranges.v_min, ranges.v_max, 12)
       kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 12)
       kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 12)
       t_int = self.float_to_uint(t_ff, ranges.t_min, ranges.t_max, 12)

       data = bytearray(8)
       data[0] = (p_int >> 8) & 0xFF
       data[1] = p_int & 0xFF
       data[2] = (v_int >> 4) & 0xFF
       data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
       data[4] = kp_int & 0xFF
       data[5] = (kd_int >> 4) & 0xFF
       data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
       data[7] = t_int & 0xFF
       return bytes(data)

   def unpack_mit_reply(self, data: bytes, ranges: MITRanges) -> Optional[Dict]:
       if len(data) != 8:
           return None

       # Reply format assumed:
       # [0]=id, [1..2]=pos, [3..4]=vel, [4..5]=torque bits, [6]=temp, [7]=error
       p_int = (data[1] << 8) | data[2]
       v_int = (data[3] << 4) | (data[4] >> 4)
       t_int = ((data[4] & 0x0F) << 8) | data[5]
       temp_raw = data[6]
       error_flag = data[7]

       pos = self.uint_to_float(p_int, P_MIN, P_MAX, 16)
       vel = self.uint_to_float(v_int, ranges.v_min, ranges.v_max, 12)
       tor = self.uint_to_float(t_int, ranges.t_min, ranges.t_max, 12)
       temp_c = int(temp_raw) - 40

       return {
           'position': pos,
           'velocity': vel,
           'effort': tor,
           'temperature': temp_c,
           'error_flag': error_flag,
           'last_update': time.time()
       }

   # ------------------------
   # Shutdown (benign)
   # ------------------------
   def shutdown(self):
       """
       Best-effort benign shutdown:
       - DISARM (kp=0) and send a short burst of SAFE commands
       - send MIT_EXIT
       - close CAN
       """
       self.get_logger().warn('Shutdown: sending SAFE burst then EXIT...')
       try:
           self._set_safe_mode()
           self._safe_burst(duration_s=0.25)
       except Exception:
           pass

       for motor_id in self.motor_ids:
           try:
               self.bus.send(can.Message(arbitration_id=motor_id, is_extended_id=False, data=MIT_EXIT))
           except can.CanError:
               self.get_logger().error(f'CAN send failed (EXIT) for motor {motor_id}')

       try:
           self.bus.shutdown()
       except Exception:
           pass


def main(args=None):
   rclpy.init(args=args)
   node = MITMotorDriver()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       # Ctrl+C should land here (SIGINT). We did not override SIGINT.
       pass
   finally:
       try:
           node._shutdown_requested = True
           node.shutdown()
       except Exception:
           pass
       node.destroy_node()
       # Safe to call even if already shutting down
       try:
           rclpy.shutdown()
       except Exception:
           pass


if __name__ == '__main__':
   main()

