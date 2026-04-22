#!/usr/bin/env python3
###################################################################################
# Author: Jinhan Lee
# Date: 2026-04-08
# Description: CANopen steering recovery and homing helper for swerve drive system
###################################################################################

import math
import select
import socket
import struct
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

try:
    import can  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    can = None

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from can_msgs.msg import Frame

DEFAULT_DRIVE_RPDO_BASES = [0x200, 0x200, 0x200, 0x200]
DEFAULT_STEER_IDS = [2, 4, 6, 8]
DEFAULT_STEER_BUS_NAMES = ['can1', 'can0', 'can1', 'can0']
DEFAULT_STEER_RPDO_BASES = [0x200, 0x200, 0x200, 0x200]

DEFAULT_NMT_NODE_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
DEFAULT_NMT_BUS_NAMES = ['can1', 'can1', 'can0', 'can0', 'can1', 'can1', 'can0', 'can0']

DRIVE_GEAR_RATIO = 20.0

OD_CONTROL_WORD = 0x6040 
OD_MODES_OF_OPERATION = 0x6060
OD_TARGET_POSITION = 0x607A
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084
OD_STATUS_WORD = 0x6041
OD_TARGET_VELOCITY = 0x60FF

# TPDO2 COB-ID base for Status Word feedback (0x280 + Node ID)
TPDO2_COB_ID_BASE = 0x280
# Status Word Bit 10: Target Reached
STATUS_WORD_TARGET_REACHED_BIT = 10

# TPDO2 매핑 관련 OD (Object Dictionary) 인덱스
OD_TPDO2_MAPPING = 0x1A01        # TPDO2 매핑 파라미터
OD_TPDO2_COMM = 0x1801           # TPDO2 통신 파라미터
# Status Word 매핑 값: 0x6041 (index) + 0x00 (subindex) + 0x10 (16비트)
STATUS_WORD_MAPPING_VALUE = 0x60410010

CAN_FRAME_FORMAT = '=IB3x8s'
NEGATIVE_JOG_VELOCITY = -536
POSITIVE_JOG_VELOCITY = 536
JOG_DURATION = 0.6
SETTLE_DURATION = 0.2
SDO_TIMEOUT = 0.8

NMT_STATE_MAP = {
    0x00: 'Boot-up',
    0x04: 'Stopped',
    0x05: 'Operational',
    0x7F: 'Pre-operational',
}


class CANError(RuntimeError):
    """Raised when a CAN operation fails or times out."""


@dataclass
class NodeStatus:
    raw: int
    ready_to_switch_on: bool
    switched_on: bool
    operation_enabled: bool
    fault: bool
    voltage_enabled: bool
    quick_stop: bool
    switch_on_disabled: bool
    warning: bool
    target_reached: bool
    position_limit_active: bool

    def summary(self) -> str:
        flags: List[str] = []
        if self.operation_enabled:
            flags.append('OpEn')
        if self.switched_on:
            flags.append('SwOn')
        if self.ready_to_switch_on:
            flags.append('Rdy')
        if self.voltage_enabled:
            flags.append('Volt')
        if self.target_reached:
            flags.append('Target')
        if self.position_limit_active:
            flags.append('Limit')
        if self.fault:
            flags.append('Fault')
        if self.warning:
            flags.append('Warn')
        if self.quick_stop:
            flags.append('QuickStop')
        if self.switch_on_disabled:
            flags.append('Disabled')
        return ', '.join(flags) if flags else '(no flags)'


class CANInterface:
    """Minimal SocketCAN helper for synchronous SDO transactions."""

    def __init__(self, name: str, logger: Any) -> None:
        self.name = name
        self._logger = logger
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        except OSError as exc:  # pragma: no cover - depends on runtime permissions
            raise CANError(f"Cannot open CAN interface '{name}': {exc}") from exc
        try:
            self.sock.bind((name,))
        except OSError as exc:
            self.sock.close()
            raise CANError(f"Cannot bind CAN interface '{name}': {exc}") from exc

    def close(self) -> None:
        self.sock.close()

    def drain(self) -> None:
        """Clear residual frames to keep request/response deterministic."""

        while True:
            readable, _, _ = select.select([self.sock], [], [], 0)
            if not readable:
                break
            self.sock.recv(16)

    def send(self, can_id: int, payload: bytes) -> None:
        if len(payload) > 8:
            raise ValueError('CAN payload cannot exceed 8 bytes')
        data = payload.ljust(8, b'\x00')
        frame = struct.pack(CAN_FRAME_FORMAT, can_id, len(data), data)
        self.sock.send(frame)

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        readable, _, _ = select.select([self.sock], [], [], timeout)
        if not readable:
            return None
        frame = self.sock.recv(16)
        can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
        return can_id, data[:dlc]

    def wait_for(self, can_id: int, timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            received = self.recv(remaining)
            if received is None:
                return None
            incoming_id, data = received
            if incoming_id == can_id:
                return data
            self._logger.debug(
                f"[{self.name}] Ignoring frame 0x{incoming_id:03X} -> {data.hex(sep=' ')}"
            )


class SteerRecoveryHelper:
    """Encapsulate steering limit recovery logic."""

    def __init__(
        self,
        logger: Any,
        can0_nodes: List[int],
        can1_nodes: List[int],
        cancel_event: threading.Event,
    ) -> None:
        self._logger = logger
        self._targets: Dict[str, List[int]] = {
            'can0': [int(node) for node in can0_nodes],
            'can1': [int(node) for node in can1_nodes],
        }
        self._cancel_event = cancel_event

    def run(self) -> None:
        interfaces: Dict[str, CANInterface] = {}
        try:
            for name in sorted(self._targets.keys()):
                interfaces[name] = CANInterface(name, self._logger)
                interfaces[name].drain()
        except CANError as exc:
            self._logger.error(f"[SteerRecovery] {exc}")
            for interface in interfaces.values():
                interface.close()
            return

        try:
            for if_name, nodes in self._targets.items():
                interface = interfaces.get(if_name)
                if not interface:
                    continue
                for node_id in nodes:
                    if self._cancel_event.is_set():
                        return
                    try:
                        self._recover_node(interface, node_id)
                    except CANError as exc:
                        if self._cancel_event.is_set():
                            self._logger.debug(
                                '[SteerRecovery][%s:%02d] cancelled (%s)',
                                if_name,
                                node_id,
                                exc,
                            )
                            return
                        self._logger.error(f"[SteerRecovery][{if_name}:{node_id:02d}] {exc}")
        finally:
            for interface in interfaces.values():
                interface.close()

        self._logger.info('[SteerRecovery] Recovery sequence complete')

    def _sleep_with_cancel(self, duration: float) -> bool:
        if duration <= 0:
            return False
        return self._cancel_event.wait(duration)

    def _sdo_upload(self, interface: CANInterface, node_id: int, index: int, sub_index: int) -> bytes:
        request = bytes([
            0x40,
            index & 0xFF,
            (index >> 8) & 0xFF,
            sub_index & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
        ])
        cob_id = 0x600 + node_id
        response_id = 0x580 + node_id
        self._logger.debug(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] SDO upload {index:04X}:{sub_index:02X}"
        )
        interface.send(cob_id, request)
        data = interface.wait_for(response_id, SDO_TIMEOUT)
        if data is None:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Timeout waiting for SDO upload {index:04X}:{sub_index:02X}"
            )
        if len(data) < 8:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Invalid SDO upload response length {len(data)}"
            )
        if data[1] != (index & 0xFF) or data[2] != ((index >> 8) & 0xFF) or data[3] != (sub_index & 0xFF):
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Unexpected object {data[2]<<8 | data[1]:04X}:{data[3]:02X}"
            )
        if (data[0] & 0xE0) != 0x40:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Unexpected SDO upload specifier 0x{data[0]:02X}"
            )
        unused_bytes = (data[0] >> 2) & 0x03
        length = 4 - unused_bytes
        return data[4:4 + length]

    def _sdo_download(
        self,
        interface: CANInterface,
        node_id: int,
        index: int,
        sub_index: int,
        value: bytes,
    ) -> None:
        size = len(value)
        if size == 1:
            command = 0x2F
        elif size == 2:
            command = 0x2B
        elif size == 4:
            command = 0x23
        else:
            raise ValueError('SDO expedited download supports 1, 2, or 4 byte writes')
        request = bytes([
            command,
            index & 0xFF,
            (index >> 8) & 0xFF,
            sub_index & 0xFF,
        ]) + value + b'\x00' * (8 - 4 - size)
        cob_id = 0x600 + node_id
        response_id = 0x580 + node_id
        self._logger.debug(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] SDO download {index:04X}:{sub_index:02X} value={value.hex(sep=' ')}"
        )
        interface.send(cob_id, request)
        data = interface.wait_for(response_id, SDO_TIMEOUT)
        if data is None:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Timeout waiting for SDO download response"
            )
        if len(data) < 8 or (data[0] & 0xE0) != 0x60:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Unexpected SDO download ack 0x{data[0]:02X}"
            )
        if data[1] != (index & 0xFF) or data[2] != ((index >> 8) & 0xFF) or data[3] != (sub_index & 0xFF):
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Ack mismatch: got {data[2]<<8 | data[1]:04X}:{data[3]:02X}"
            )

    def _decode_status_word(self, raw: int) -> NodeStatus:
        return NodeStatus(
            raw=raw,
            ready_to_switch_on=bool(raw & (1 << 0)),
            switched_on=bool(raw & (1 << 1)),
            operation_enabled=bool(raw & (1 << 2)),
            fault=bool(raw & (1 << 3)),
            voltage_enabled=bool(raw & (1 << 4)),
            quick_stop=bool(raw & (1 << 5)),
            switch_on_disabled=bool(raw & (1 << 6)),
            warning=bool(raw & (1 << 7)),
            target_reached=bool(raw & (1 << 10)),
            position_limit_active=bool(raw & (1 << 11)),
        )

    def _read_status_word(self, interface: CANInterface, node_id: int) -> NodeStatus:
        payload = self._sdo_upload(interface, node_id, OD_STATUS_WORD, 0x00)
        if len(payload) != 2:
            raise CANError(
                f"[{interface.name}:{node_id:02d}] Unexpected status word payload length {len(payload)}"
            )
        raw = payload[0] | (payload[1] << 8)
        status = self._decode_status_word(raw)
        self._logger.info(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Status word 0x{raw:04X} -> {status.summary()}"
        )
        return status

    def _write_control_word(self, interface: CANInterface, node_id: int, value: int) -> None:
        self._sdo_download(interface, node_id, OD_CONTROL_WORD, 0x00, struct.pack('<H', value))
        self._logger.debug(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Control word set to 0x{value:04X}"
        )

    def _write_target_velocity(self, interface: CANInterface, node_id: int, value: int) -> None:
        self._sdo_download(interface, node_id, OD_TARGET_VELOCITY, 0x00, struct.pack('<i', value))
        self._logger.debug(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Target velocity set to {value}"
        )

    def _jog_node(self, interface: CANInterface, node_id: int, velocity: int) -> NodeStatus:
        self._write_target_velocity(interface, node_id, velocity)
        self._logger.info(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Jogging at {velocity} pulse/s for {JOG_DURATION:.2f} s"
        )
        if self._sleep_with_cancel(JOG_DURATION):
            raise CANError(f'[{interface.name}:{node_id:02d}] Jog cancelled')
        self._write_target_velocity(interface, node_id, 0)
        self._logger.info(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Jog stopped, wait {SETTLE_DURATION:.2f} s for settle"
        )
        if self._sleep_with_cancel(SETTLE_DURATION):
            raise CANError(f'[{interface.name}:{node_id:02d}] Settle cancelled')
        return self._read_status_word(interface, node_id)

    def _reset_fault(self, interface: CANInterface, node_id: int) -> None:
        self._logger.info(f"[SteerRecovery][{interface.name}:{node_id:02d}] Fault reset (bit7 pulse)")
        self._write_control_word(interface, node_id, 0x0080)
        if self._sleep_with_cancel(0.1):
            raise CANError(f'[{interface.name}:{node_id:02d}] Fault reset cancelled')
        self._write_control_word(interface, node_id, 0x0000)
        if self._sleep_with_cancel(0.1):
            raise CANError(f'[{interface.name}:{node_id:02d}] Fault reset cancelled')

    def _reenable_operation(self, interface: CANInterface, node_id: int) -> None:
        self._logger.info(f"[SteerRecovery][{interface.name}:{node_id:02d}] Transition to Operation Enable")
        self._write_control_word(interface, node_id, 0x0006)
        if self._sleep_with_cancel(0.05):
            raise CANError(f'[{interface.name}:{node_id:02d}] Enable cancelled')
        self._write_control_word(interface, node_id, 0x0007)
        if self._sleep_with_cancel(0.05):
            raise CANError(f'[{interface.name}:{node_id:02d}] Enable cancelled')
        self._write_control_word(interface, node_id, 0x000F)
        if self._sleep_with_cancel(0.1):
            raise CANError(f'[{interface.name}:{node_id:02d}] Enable cancelled')

    def _recover_node(self, interface: CANInterface, node_id: int) -> None:
        self._logger.info(f"[SteerRecovery][{interface.name}:{node_id:02d}] ========== Checking steer motor status ==========")
        status = self._read_status_word(interface, node_id)
        
        # Detailed initial status logging
        self._log_detailed_status(interface.name, node_id, status, "Initial")
        
        needs_release = status.position_limit_active
        needs_fault_reset = status.fault
        needs_enable = not status.operation_enabled

        # Check for any issues and log warnings
        if needs_release:
            self._logger.warning(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Position limit active. Axis movement restricted."
            )
        if needs_fault_reset:
            self._logger.error(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Drive fault detected. Fault reset required."
            )
        if needs_enable:
            self._logger.warning(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Operation Enable missing. Drive power stage disabled."
            )
        if status.warning:
            self._logger.warning(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Internal warning flag active in drive statusword."
            )

        if not needs_release and not needs_fault_reset and status.operation_enabled:
            self._logger.info(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Node status nominal: Operation Enabled, no active faults."
            )
            return

        if needs_release:
            self._logger.info(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Initiating limit recovery: negative jog."
            )
            status = self._jog_node(interface, node_id, NEGATIVE_JOG_VELOCITY)
            if status.position_limit_active:
                self._logger.info(
                    f"[SteerRecovery][{interface.name}:{node_id:02d}] Limit persists: positive jog."
                )
                status = self._jog_node(interface, node_id, POSITIVE_JOG_VELOCITY)
            
            if status.position_limit_active:
                self._logger.error(
                    f"[SteerRecovery][{interface.name}:{node_id:02d}] Limit recovery failed. Manual intervention required."
                )
            else:
                self._logger.info(
                    f"[SteerRecovery][{interface.name}:{node_id:02d}] Position limit cleared."
                )

        if needs_fault_reset or status.fault:
            self._logger.info(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Executing fault reset sequence."
            )
            self._reset_fault(interface, node_id)
            status = self._read_status_word(interface, node_id)
            
            if status.fault:
                self._logger.error(
                    f"[SteerRecovery][{interface.name}:{node_id:02d}] Fault persists post-reset. Hardware inspection required."
                )
            else:
                self._logger.info(
                    f"[SteerRecovery][{interface.name}:{node_id:02d}] Fault cleared."
                )

        self._reenable_operation(interface, node_id)
        final_status = self._read_status_word(interface, node_id)
        
        # Final status logging with detailed summary
        self._log_detailed_status(interface.name, node_id, final_status, "Final")
        
        if final_status.fault or final_status.position_limit_active or not final_status.operation_enabled:
            self._logger.error(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Recovery sequence incomplete. Final status: 0x{final_status.raw:04X} ({final_status.summary()})"
            )
        else:
            self._logger.info(
                f"[SteerRecovery][{interface.name}:{node_id:02d}] Recovery sequence complete. Node operational."
            )

    def _log_detailed_status(self, if_name: str, node_id: int, status: NodeStatus, label: str) -> None:
        """Log detailed status information for a motor."""
        status_lines = [
            f"[SteerRecovery][{if_name}:{node_id:02d}] {label} Status (0x{status.raw:04X}):",
            f"  - Operation Enabled     : {'TRUE' if status.operation_enabled else 'FALSE'}",
            f"  - Switched On           : {'TRUE' if status.switched_on else 'FALSE'}",
            f"  - Ready to Switch On    : {'TRUE' if status.ready_to_switch_on else 'FALSE'}",
            f"  - Fault                 : {'ACTIVE' if status.fault else 'INACTIVE'}",
            f"  - Position Limit        : {'ACTIVE' if status.position_limit_active else 'INACTIVE'}",
            f"  - Warning               : {'ACTIVE' if status.warning else 'INACTIVE'}",
            f"  - Target Reached        : {'TRUE' if status.target_reached else 'FALSE'}",
            f"  - Voltage Enabled       : {'TRUE' if status.voltage_enabled else 'FALSE'}",
        ]
        for line in status_lines:
            self._logger.info(line)

    def _write_target_position(self, interface: CANInterface, node_id: int, position: int) -> None:
        """Write Target Position (0x607A) via SDO."""
        self._sdo_download(interface, node_id, OD_TARGET_POSITION, 0x00, struct.pack('<i', position))
        self._logger.debug(
            f"[SteerRecovery][{interface.name}:{node_id:02d}] Target position set to {position}"
        )

    def _home_node_to_zero(self, interface: CANInterface, node_id: int) -> bool:
        """
        Move a single steer motor to 0 degrees (front-facing position).
        
        Returns:
            True if homing was successful, False otherwise.
        """
        try:
            self._logger.info(f"[SteerHoming][{interface.name}:{node_id:02d}] Moving to 0 degrees (front-facing)")
            
            # 1. Set Target Position to 0
            self._write_target_position(interface, node_id, 0)
            
            # 2. Trigger position command with New Setpoint bit (bit 4)
            # Control Word: 0x001F = Enable + New Setpoint
            self._write_control_word(interface, node_id, 0x001F)
            if self._sleep_with_cancel(0.05):
                return False
            
            # 3. Clear New Setpoint bit
            self._write_control_word(interface, node_id, 0x000F)
            
            # 4. Wait for movement to complete
            if self._sleep_with_cancel(0.5):
                return False
            
            # 5. Verify status
            status = self._read_status_word(interface, node_id)
            if status.target_reached:
                self._logger.info(
                    f"[SteerHoming][{interface.name}:{node_id:02d}] Homing complete - target reached"
                )
                return True
            else:
                self._logger.warning(
                    f"[SteerHoming][{interface.name}:{node_id:02d}] Homing may not be complete - "
                    f"target_reached={status.target_reached}"
                )
                return True  # Still consider it successful as the command was sent
                
        except CANError as exc:
            self._logger.error(f"[SteerHoming][{interface.name}:{node_id:02d}] Homing failed: {exc}")
            return False

    def home_to_zero(self) -> bool:
        """
        Move all steer motors to 0 degrees (front-facing position).
        
        This should be called after recovery is complete and before/after normal operation.
        
        Returns:
            True if all motors were successfully homed, False if any failed.
        """
        self._logger.info('[SteerHoming] Starting homing sequence - moving all wheels to front-facing (0 deg)')
        
        interfaces: Dict[str, CANInterface] = {}
        all_success = True
        
        try:
            # Open CAN interfaces
            for name in sorted(self._targets.keys()):
                try:
                    interfaces[name] = CANInterface(name, self._logger)
                    interfaces[name].drain()
                except CANError as exc:
                    self._logger.error(f"[SteerHoming] Failed to open {name}: {exc}")
                    all_success = False
                    continue
            
            # Home each motor
            for if_name, nodes in self._targets.items():
                interface = interfaces.get(if_name)
                if not interface:
                    all_success = False
                    continue
                    
                for node_id in nodes:
                    if self._cancel_event.is_set():
                        self._logger.info('[SteerHoming] Homing cancelled')
                        return False
                    
                    if not self._home_node_to_zero(interface, node_id):
                        all_success = False
            
            # Wait for all motors to settle
            if not self._cancel_event.is_set():
                self._logger.info('[SteerHoming] Waiting for all motors to settle...')
                self._sleep_with_cancel(0.3)
            
        finally:
            # Close all interfaces
            for interface in interfaces.values():
                interface.close()
        
        if all_success:
            self._logger.info('[SteerHoming] All wheels successfully moved to front-facing position (0 deg)')
        else:
            self._logger.warning('[SteerHoming] Some wheels may not have been homed successfully')
        
        return all_success


class IntegratedSwerveController0930(Node):
    def __init__(self) -> None:
        super().__init__('integrated_swerve_controller')
        self.get_logger().info('--- integrated swerve controller 0930 ---')

        self.declare_parameter('to_can_bus_topic_can0', '/can0/to_can_bus')
        self.declare_parameter('to_can_bus_topic_can1', '/can1/to_can_bus')
        self.declare_parameter('from_can_bus_topic_can0', '/can0/from_can_bus')
        self.declare_parameter('from_can_bus_topic_can1', '/can1/from_can_bus')
        # [2025-11-27] RPDO SYNC 동기화 사용 시 이 값이 실제 제어 주기가 됨
        # 기존 0.5초 → 0.02초(50Hz)로 변경하여 실시간 제어 가능하게 함
        self.declare_parameter('heartbeat_sync_period_sec', 0.02)  # 단위: 초 (s)
        self.declare_parameter('can_feedback_timeout_sec', 3.0)  # 단위: 초 (s)
        self.declare_parameter('feedback_watchdog_check_period_sec', 0.1)  # 단위: 초 (s)

        self._declare_swerve_parameters()
        self._declare_drive_parameters()
        self._declare_steer_parameters()
        self._declare_support_parameters()

        self._load_swerve_params()
        self._load_drive_params()
        self._load_steer_params()
        self._load_support_params()

        self._all_nodes_config: Dict[int, str] = {}
        for idx, node_id in enumerate(self.drive_motor_ids):
            self._all_nodes_config[int(node_id)] = self.drive_motor_bus_names[idx]
        for idx, node_id in enumerate(self.steer_motor_ids):
            self._all_nodes_config[int(node_id)] = self.steer_motor_bus_names[idx]
        self.get_logger().info(f"All managed nodes: {self._all_nodes_config}")

        can0_topic = self.get_parameter('to_can_bus_topic_can0').value
        can1_topic = self.get_parameter('to_can_bus_topic_can1').value
        self.can0_pub = self.create_publisher(Frame, can0_topic, 10)
        self.can1_pub = self.create_publisher(Frame, can1_topic, 10)

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.drive_rpm_pub = self.create_publisher(Float32MultiArray, '/drive_rpms', qos_reliable)
        self.steer_angle_pub = self.create_publisher(Float32MultiArray, '/steer_angles', qos_reliable)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_reliable)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', qos_reliable)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # URDF 파일에 정의된 관절 이름 (조향 4개 + 구동 4개)
        self.joint_names = [
            'fl_base_to_steering', 'fr_base_to_steering', 'rl_base_to_steering', 'rr_base_to_steering',
            'fl_steering_to_wheel', 'fr_steering_to_wheel', 'rl_steering_to_wheel', 'rr_steering_to_wheel'
        ]
        # 구동 바퀴의 회전 위치(라디안)를 누적할 리스트
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]  
        
        cmd_vel_reliability = str(self.get_parameter('cmd_vel.reliability').value).lower()
        if cmd_vel_reliability not in ('reliable', 'best_effort'):
            self.get_logger().warn("cmd_vel.reliability value is invalid, setting to 'reliable'.")
            cmd_vel_reliability = 'reliable'
        if cmd_vel_reliability == 'best_effort':
            cmd_vel_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            cmd_vel_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        # [2025-11-26] cmd_vel 토픽 구독 (Safety layer와의 통합)
        # - Safety layer 사용 시: launch 파일에서 cmd_vel_topic:='/cmd_vel_safe' 전달
        # - Safety layer 미사용 시: 기본값 '/cmd_vel' 사용 (이전 동작 복구)
        # 체인: Nav2 → /cmd_vel → (Emergency Stop) → /cmd_vel_safe → Motor
        self.cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, cmd_vel_qos)
        self.get_logger().info(f"[Safety] cmd_vel subscription topic: {self.cmd_vel_topic}")

        self.create_subscription(
            Float32MultiArray,
            '/manual_steer_override',
            self.manual_steer_override_callback,
            qos_reliable,
        )
        self.get_logger().info("[Manual] steer override subscription topic: /manual_steer_override")

        from_can0_topic = self.get_parameter('from_can_bus_topic_can0').value
        from_can1_topic = self.get_parameter('from_can_bus_topic_can1').value
        self.create_subscription(Frame, from_can0_topic, self._can_feedback_callback, 10)
        self.create_subscription(Frame, from_can1_topic, self._can_feedback_callback, 10)

        self.can_feedback_timeout = float(self.get_parameter('can_feedback_timeout_sec').value)
        watchdog_check_period = float(self.get_parameter('feedback_watchdog_check_period_sec').value)
        self.last_can_feedback_time = self.get_clock().now()
        self._is_feedback_watchdog_tripped = False
        self._is_shutting_down = False  # Flag to ignore cmd_vel during shutdown
        self.feedback_watchdog_timer = None
        if self.can_feedback_timeout > 0.0:
            period = watchdog_check_period if watchdog_check_period > 0.0 else 0.1
            self.feedback_watchdog_timer = self.create_timer(period, self._feedback_watchdog_callback)
            self.get_logger().info(
                f"/from_can_bus watchdog: timeout {self.can_feedback_timeout:.2f}s, check period {period:.2f}s"
            )

        self.heartbeat_period = float(self.get_parameter('heartbeat_sync_period_sec').value)
        self.heartbeat_timer = None
        if self.heartbeat_period > 0.0:
            self.heartbeat_timer = self.create_timer(self.heartbeat_period, self._send_sync_heartbeat)
            self.get_logger().info(
                f"CANopen SYNC heartbeat: period {self.heartbeat_period:.2f}s"
            )

        self._init_sequence = [0x0080, 0x0006, 0x0007, self.drive_control_word]
        self._init_sequence_step = 0
        self._init_timer = None
        
        # [2025-11-27] Drive 모터 RPDO SYNC 동기화 설정
        self._initialize_drive_rpdo_sync()
        
        if self.auto_enable_sequence:
            self._init_timer = self.create_timer(0.2, self._run_drive_init_step)

        self._initialize_steer_motors()
        self._retrigger_timer = None
        if self.retrigger_period > 0.0:
            self._retrigger_timer = self.create_timer(self.retrigger_period, self._retrigger_publish)

        self._support_thread_stop = threading.Event()
        self._support_thread: Optional[threading.Thread] = None
        if self._support_enable and (self._nmt_enable or self._steer_recovery_enable):
            self._support_thread = threading.Thread(
                target=self._run_support_procedures,
                name='startup_support_tasks',
                daemon=True,
            )
            self._support_thread.start()

        # [2025-11-28] 조향 완료 대기 기능: 4개 조향 모터의 Target Reached 상태 추적
        # TPDO2 (0x280 + Node ID)로 전송되는 Position 또는 Status Word를 모니터링
        self._steer_target_reached: Dict[int, bool] = {
            int(node_id): False for node_id in self.steer_motor_ids
        }
        # 조향 모터 Node ID → TPDO2 COB-ID 매핑
        self._steer_tpdo2_cob_ids: Dict[int, int] = {
            TPDO2_COB_ID_BASE + int(node_id): int(node_id) 
            for node_id in self.steer_motor_ids
        }
        # 조향 명령 전송 시간 (타임아웃 계산용)
        self._last_steer_command_time: Optional[Any] = None
        # 첫 번째 조향 명령 수신 전에는 게이팅 비활성화
        self._steering_gate_active = False
        # 큰 조향 변화가 발생하여 대기 중인지 여부
        self._steering_waiting_for_large_change = False
        # TPDO2 Position 기반 판단을 위한 변수
        # 각 모터의 현재 위치 (TPDO2에서 수신)
        self._steer_actual_position: Dict[int, int] = {
            int(node_id): 0 for node_id in self.steer_motor_ids
        }
        # 각 모터의 목표 위치 (마지막으로 전송한 명령)
        self._steer_target_position: Dict[int, int] = {
            int(node_id): 0 for node_id in self.steer_motor_ids
        }
        self.get_logger().info(
            f"[Steering] Steering motor TPDO2 COB-IDs: "
            f"{[hex(cob_id) for cob_id in self._steer_tpdo2_cob_ids.keys()]}"
        )

        self.get_logger().info(f'--- waiting for {self.cmd_vel_topic} ---')

    def _declare_swerve_parameters(self) -> None:
        self.declare_parameter('robot.wheel_base', 1.10)  # 단위: 미터 (m)
        self.declare_parameter('robot.track_width', 0.705)  # 단위: 미터 (m)
        self.declare_parameter('robot.track_front', 0.88)  # 단위: 미터 (m)
        self.declare_parameter('robot.track_rear', 0.7)  # 단위: 미터 (m)
        self.declare_parameter('robot.wheel_radius', 0.075)  # 단위: 미터 (m)
        self.declare_parameter('limits.max_linear_speed', 2.0)  # 단위: 미터/초 (m/s)
        self.declare_parameter('limits.max_angular_speed', 10.0)  # 단위: 라디안/초 (rad/s)
        self.declare_parameter('limits.max_wheel_rpm', 3000.0)  # 단위: RPM
        self.declare_parameter('limits.max_accel', 0.8)  # 단위: 미터/초² (m/s²)
        self.declare_parameter('limits.max_angular_accel', 1.57)  # 단위: 라디안/초² (rad/s²) (90도/초²)
        self.declare_parameter('steer.max_mech_angle_deg', 140.0)  # 단위: 도 (deg)
        self.declare_parameter('steer.flip_hysteresis_deg', 5.0)  # 단위: 도 (deg)
        self.declare_parameter('steer.enable_flip', False)  # 단위: 없음 (bool)
        self.declare_parameter('steer.offset_deg', [0.0, 0.0, 0.0, 0.0])  # 단위: 도 (deg) 리스트
        self.declare_parameter('cmd_vel.reliability', 'best_effort')  # 단위: 없음 (문자열)
        self.declare_parameter('debug.print_intermediate', False)  # 단위: 없음 (bool)
        self.declare_parameter('drive.invert_modules', [False, True, False, True])  # 단위: 없음 (bool 리스트)
        
        # 조향-주행 협조 제어 파라미터
        self.declare_parameter('coordinated_control.enable', False)  # 단위: 없음 (bool)
        self.declare_parameter('coordinated_control.full_speed_angle_error_deg', 5.0)  # 단위: 도 (deg)
        self.declare_parameter('coordinated_control.zero_speed_angle_error_deg', 45.0)  # 단위: 도 (deg)
        self.declare_parameter('coordinated_control.min_speed_ratio', 0.1)  # 단위: 없음 (비율, 0.0~1.0)
        self.declare_parameter('coordinated_control.use_smooth_curve', True)  # 단위: 없음 (bool)    


    def _load_swerve_params(self) -> None:
        self.L = float(self.get_parameter('robot.wheel_base').value)
        self.W = float(self.get_parameter('robot.track_width').value)
        self.track_front = float(self.get_parameter('robot.track_front').value)
        self.track_rear = float(self.get_parameter('robot.track_rear').value)
        if self.track_front <= 0.0:
            self.get_logger().warn('robot.track_front <= 0.0 → track_width used')
            self.track_front = float(self.W)
        if self.track_rear <= 0.0:
            self.get_logger().warn('robot.track_rear <= 0.0 → track_width used')
            self.track_rear = float(self.W)
        self.r = float(self.get_parameter('robot.wheel_radius').value)
        self.vmax = float(self.get_parameter('limits.max_linear_speed').value)
        self.wmax = float(self.get_parameter('limits.max_angular_speed').value)
        self.wwheel_rad_s = (
            float(self.get_parameter('limits.max_wheel_rpm').value) * 2.0 * math.pi / 60.0
        )
        self.a_max = float(self.get_parameter('limits.max_accel').value)
        self.alpha_max = float(self.get_parameter('limits.max_angular_accel').value)

        self.STEER_LIMIT_RAD = abs(float(self.get_parameter('steer.max_mech_angle_deg').value)) * math.pi / 180.0
        self.FLIP_HYST_RAD = abs(float(self.get_parameter('steer.flip_hysteresis_deg').value)) * math.pi / 180.0
        self._flip_enabled = bool(self.get_parameter('steer.enable_flip').value)
        offset_deg_list = list(self.get_parameter('steer.offset_deg').value)
        if len(offset_deg_list) < 4:
            offset_deg_list.extend([0.0] * (4 - len(offset_deg_list)))
        self._steer_offset_rad = [math.radians(float(x)) for x in offset_deg_list]

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_omega = 0.0
        self.prev_angles_raw: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.last_msg_time = self.get_clock().now()
        self._module_labels = ['FL', 'FR', 'RL', 'RR']
        self._debug_enabled = bool(self.get_parameter('debug.print_intermediate').value)
        invert_list = list(self.get_parameter('drive.invert_modules').value)
        if len(invert_list) < 4:
            invert_list.extend([False] * (4 - len(invert_list)))
        self._drive_invert_flags = [bool(v) for v in invert_list[:4]]

        half_base = self.L / 2.0
        half_front = self.track_front / 2.0
        half_rear = self.track_rear / 2.0
        self._module_positions: List[Tuple[float, float]] = [
            (half_base, half_front),
            (half_base, -half_front),
            (-half_base, half_rear),
            (-half_base, -half_rear),
        ]
        layout_info = ', '.join(
            f'{label}=({pos[0]:+.3f},{pos[1]:+.3f})'
            for label, pos in zip(self._module_labels, self._module_positions)
        )

        self.get_logger().info(f'Module geometry (x,y m): {layout_info}')
        if not self._flip_enabled:
            self.get_logger().info('Steer flip optimization disabled.')

        self._coord_control_enable = bool(
            self.get_parameter('coordinated_control.enable').value
        )
        self._coord_full_speed_error = math.radians(float(
            self.get_parameter('coordinated_control.full_speed_angle_error_deg').value
        ))
        self._coord_zero_speed_error = math.radians(float(
            self.get_parameter('coordinated_control.zero_speed_angle_error_deg').value
        ))
        self._coord_min_speed = float(
            self.get_parameter('coordinated_control.min_speed_ratio').value
        )
        self._coord_use_smooth = bool(
            self.get_parameter('coordinated_control.use_smooth_curve').value
        )
        
            # 로그 출력
        if self._coord_control_enable:
            self.get_logger().info(
                f"Steering-Driving Coordinated Control Enabled: "
                f"full_speed={math.degrees(self._coord_full_speed_error):.1f}°, "
                f"zero_speed={math.degrees(self._coord_zero_speed_error):.1f}°, "
                f"min_ratio={self._coord_min_speed:.1%}, "
                f"smooth={'S-curve' if self._coord_use_smooth else 'Linear'}"
            )
        else:
            self.get_logger().info("Steering-Driving Coordinated Control Disabled")

    def _declare_drive_parameters(self) -> None:
        self.declare_parameter('drive_motor_ids', [1, 3, 5, 7])  # 단위: 없음 (정수 리스트)
        self.declare_parameter('motor_direction_multipliers', [1.0, 1.0, 1.0, 1.0])  # 단위: 없음 (배수 리스트)
        self.declare_parameter('target_velocity_pulses_per_rev', 10000.0)  # 단위: pulse/rev
        self.declare_parameter('drive_motor_bus_names', ['can1', 'can0', 'can1', 'can0'])  # 단위: 없음 (문자열 리스트)
        self.declare_parameter('drive_motor_rpdo_bases', DEFAULT_DRIVE_RPDO_BASES)  # 단위: 없음 (16진수 리스트)
        self.declare_parameter('drive_control_word', 0x000F)  # 단위: 없음 (16진수)
        self.declare_parameter('drive_mode_of_operation', 0x03)  # 단위: 없음 (정수)
        self.declare_parameter('auto_enable_sequence', True)  # 단위: 없음 (bool)

    def _load_drive_params(self) -> None:
        self.drive_motor_ids = list(self.get_parameter('drive_motor_ids').value)
        self.motor_directions = list(self.get_parameter('motor_direction_multipliers').value)
        if len(self.motor_directions) < len(self.drive_motor_ids):
            self.motor_directions.extend([1.0] * (len(self.drive_motor_ids) - len(self.motor_directions)))
        self.motor_directions = self.motor_directions[:len(self.drive_motor_ids)]

        self.TARGET_VEL_PPR = float(self.get_parameter('target_velocity_pulses_per_rev').value)
        self.drive_motor_bus_names = list(self.get_parameter('drive_motor_bus_names').value)
        if len(self.drive_motor_bus_names) < len(self.drive_motor_ids):
            self.get_logger().warn('drive_motor_bus_names length is insufficient, filling with can0.')
            self.drive_motor_bus_names.extend(
                ['can0'] * (len(self.drive_motor_ids) - len(self.drive_motor_bus_names))
            )
        self.drive_motor_bus_names = self.drive_motor_bus_names[:len(self.drive_motor_ids)]

        bases_param = list(self.get_parameter('drive_motor_rpdo_bases').value)
        self.drive_motor_rpdo_bases = [int(base) for base in bases_param[:len(self.drive_motor_ids)]]
        if len(self.drive_motor_rpdo_bases) < len(self.drive_motor_ids):
            deficit = len(self.drive_motor_ids) - len(self.drive_motor_rpdo_bases)
            self.get_logger().warn('drive_motor_rpdo_bases length is insufficient, filling with 0x200.')
            self.drive_motor_rpdo_bases.extend([0x200] * deficit)

        self.drive_control_word = int(self.get_parameter('drive_control_word').value)
        if not (0 <= self.drive_control_word <= 0xFFFF):
            self.get_logger().warn('drive_control_word is out of range, setting to 0x000F.')
            self.drive_control_word = 0x000F

        self.drive_mode_of_operation = int(self.get_parameter('drive_mode_of_operation').value)
        if not (-128 <= self.drive_mode_of_operation <= 127):
            self.get_logger().warn('drive_mode_of_operation is out of range, setting to 0x03.')
            self.drive_mode_of_operation = 0x03

        self.auto_enable_sequence = bool(self.get_parameter('auto_enable_sequence').value)

    def _declare_steer_parameters(self) -> None:
        self.declare_parameter('steer_motor_ids', DEFAULT_STEER_IDS)  # 단위: 없음 (정수 리스트)
        self.declare_parameter('steer_motor_bus_names', DEFAULT_STEER_BUS_NAMES)  # 단위: 없음 (문자열 리스트)
        self.declare_parameter('steer_motor_rpdo_bases', DEFAULT_STEER_RPDO_BASES)  # 단위: 없음 (16진수 리스트)
        self.declare_parameter('steer_invert_ids', [])  # 단위: 없음 (정수 리스트)
        self.declare_parameter('steer_ticks_per_rev', 0.0)  # 단위: tick/rev
        self.declare_parameter('steer_position_units', 'puu')  # 단위: 없음 (문자열)
        self.declare_parameter('steer_feed_constant_puu_per_rev', 18000.0)  # 단위: puu/rev
        self.declare_parameter('steer_elec_gear_numerator', 1.0)  # 단위: 없음 (배수)
        self.declare_parameter('steer_elec_gear_denominator', 1.0)  # 단위: 없음 (배수)
        self.declare_parameter('steer_gear_ratio', 122.5)  # 단위: 없음 (배수)
        self.declare_parameter('steer_mode_of_operation', 0x01)  # 단위: 없음 (정수)
        self.declare_parameter('steer_trigger_mode', 'pulse')  # 단위: 없음 (문자열)
        self.declare_parameter('steer_use_bit5', True)  # 단위: 없음 (bool)
        self.declare_parameter('steer_control_word_enable', 0x000F)  # 단위: 없음 (16진수)
        # Nav2 지령 변화에 빠르게 대응하기 위해 프로파일 속도/가속도/감속도 상향
        self.declare_parameter('profile_velocity', 800000)  # 단위: pulse/s
        self.declare_parameter('profile_accel', 600000)  # 단위: pulse/s²
        self.declare_parameter('profile_decel', 600000)  # 단위: pulse/s²
        self.declare_parameter('servo_on_sleep_sec', 0.05)  # 단위: 초 (s)
        self.declare_parameter('trigger_pulse_delay_sec', 0.02)  # 단위: 초 (s)
        self.declare_parameter('retrigger_period_sec', 0.0)  # 단위: 초 (s)
        
        # enable: 기능 활성화 (False면 기존 동작 유지)
        # timeout_sec: 조향 명령 후 이 시간 내에 Target Reached 안 되면 구동 허용 (0이면 무한 대기)
        # position_tolerance: TPDO2 Position 기반 판단 시 허용 오차 (pulse 단위)
        # use_position_feedback: True=TPDO2 Position 사용, False=Status Word Bit10 사용
        # large_error_threshold: "현재 위치와 목표 위치의 차이"가 이 값 이상이면 대기 (pulse 단위)
        #   - 275000 pulse ≈ 약 45도 (6125 pulse/degree 기준)
        #   - 큰 조향 오차(예: 90도 회전)는 대기 후 구동
        #   - 작은 조향 오차(경로 추적 중)는 구동하면서 조향 조정
        self.declare_parameter('steering_gate.enable', True)  # 단위: 없음 (bool)
        self.declare_parameter('steering_gate.timeout_sec', 0.0)  # 단위: 초 (s), 0 = 무한 대기
        self.declare_parameter('steering_gate.position_tolerance', 500)  # 단위: pulse, 허용 오차 500 pulse (약 0.08도)
        self.declare_parameter('steering_gate.use_position_feedback', True)  # 단위: 없음 (bool), Position 기반 판단
        self.declare_parameter('steering_gate.large_error_threshold', 275000)  # 단위: pulse, 큰 조향 오차 임계값 (45도)

    def _declare_support_parameters(self) -> None:
        self.declare_parameter('support.enable_startup_tasks', True)  # 단위: 없음 (bool)
        self.declare_parameter('support.nmt_auto_start.enable', True)  # 단위: 없음 (bool)
        self.declare_parameter('support.nmt_auto_start.node_ids', DEFAULT_NMT_NODE_IDS)  # 단위: 없음 (정수 리스트)
        self.declare_parameter('support.nmt_auto_start.bus_names', DEFAULT_NMT_BUS_NAMES)  # 단위: 없음 (문자열 리스트)
        self.declare_parameter('support.nmt_auto_start.heartbeat_duration_sec', 1.5)  # 단위: 초 (s)
        self.declare_parameter('support.nmt_auto_start.log_heartbeats', False)  # 단위: 없음 (bool)
        self.declare_parameter('support.steer_recovery.enable', True)  # 단위: 없음 (bool)
        self.declare_parameter('support.steer_recovery.can0_nodes', [4, 8])  # 단위: 없음 (정수 리스트)
        self.declare_parameter('support.steer_recovery.can1_nodes', [2, 6])  # 단위: 없음 (정수 리스트)

    def _load_steer_params(self) -> None:
        self.steer_motor_ids = list(self.get_parameter('steer_motor_ids').value)
        self.steer_motor_bus_names = list(self.get_parameter('steer_motor_bus_names').value)
        if len(self.steer_motor_bus_names) < len(self.steer_motor_ids):
            self.get_logger().warn('steer_motor_bus_names length is insufficient, filling with can0.')
            self.steer_motor_bus_names.extend(
                ['can0'] * (len(self.steer_motor_ids) - len(self.steer_motor_bus_names))
            )
        self.steer_motor_bus_names = self.steer_motor_bus_names[:len(self.steer_motor_ids)]

        bases_param = list(self.get_parameter('steer_motor_rpdo_bases').value)
        self.steer_motor_rpdo_bases = [int(v) for v in bases_param[:len(self.steer_motor_ids)]]
        if len(self.steer_motor_rpdo_bases) < len(self.steer_motor_ids):
            deficit = len(self.steer_motor_ids) - len(self.steer_motor_rpdo_bases)
            self.get_logger().warn('steer_motor_rpdo_bases 길이가 부족해 0x200을 채웁니다.')
            self.steer_motor_rpdo_bases.extend([0x200] * deficit)

        self.steer_invert_ids = set(int(v) for v in (self.get_parameter('steer_invert_ids').value or []))
        self.steer_ticks_per_rev = float(self.get_parameter('steer_ticks_per_rev').value)
        self.steer_position_units = str(self.get_parameter('steer_position_units').value).lower()
        self.steer_feed_constant = float(self.get_parameter('steer_feed_constant_puu_per_rev').value)
        self.steer_elec_gear_num = float(self.get_parameter('steer_elec_gear_numerator').value)
        self.steer_elec_gear_den = float(self.get_parameter('steer_elec_gear_denominator').value)
        if self.steer_elec_gear_den == 0.0:
            self.get_logger().warn('steer_elec_gear_denominator is 0, setting to 1.')
            self.steer_elec_gear_den = 1.0

        self.steer_gear_ratio = float(self.get_parameter('steer_gear_ratio').value)
        self.steer_mode_of_operation = int(self.get_parameter('steer_mode_of_operation').value)
        self.steer_trigger_mode = str(self.get_parameter('steer_trigger_mode').value).lower()
        self.steer_use_bit5 = bool(self.get_parameter('steer_use_bit5').value)
        self.steer_control_word_enable = int(self.get_parameter('steer_control_word_enable').value)
        if not (0 <= self.steer_control_word_enable <= 0xFFFF):
            self.get_logger().warn('steer_control_word_enable is out of range, setting to 0x000F.')
            self.steer_control_word_enable = 0x000F

        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.profile_accel = int(self.get_parameter('profile_accel').value)
        self.profile_decel = int(self.get_parameter('profile_decel').value)
        self.servo_on_sleep = float(self.get_parameter('servo_on_sleep_sec').value)
        self.trigger_pulse_delay = float(self.get_parameter('trigger_pulse_delay_sec').value)
        self.retrigger_period = float(self.get_parameter('retrigger_period_sec').value)
        if self.servo_on_sleep < 0.0:
            self.get_logger().warn('servo_on_sleep_sec is negative, setting to 0.')
            self.servo_on_sleep = 0.0
        if self.trigger_pulse_delay < 0.0:
            self.get_logger().warn('trigger_pulse_delay_sec is negative, setting to 0.')
            self.trigger_pulse_delay = 0.0
        if self.retrigger_period < 0.0:
            self.get_logger().warn('retrigger_period_sec is negative, setting to 0.')
            self.retrigger_period = 0.0

        if self.steer_position_units not in ('puu', 'ticks'):
            self.get_logger().warn("steer_position_units is not supported, setting to 'puu'.")
            self.steer_position_units = 'puu'

        if self.steer_position_units == 'puu' and self.steer_feed_constant > 0.0:
            puu_per_rev = self.steer_feed_constant * (self.steer_elec_gear_num / self.steer_elec_gear_den)
            self._pulses_per_degree = (puu_per_rev * self.steer_gear_ratio) / 360.0
            units_label = 'puu'
        else:
            if self.steer_position_units == 'puu' and self.steer_feed_constant <= 0.0:
                self.get_logger().warn('steer_feed_constant_puu_per_rev is 0 or negative, using ticks conversion.')
            if self.steer_ticks_per_rev <= 0.0:
                self.get_logger().warn('steer_ticks_per_rev is 0, setting to 1.')
                self.steer_ticks_per_rev = 1.0
            self._pulses_per_degree = (self.steer_ticks_per_rev * self.steer_gear_ratio) / 360.0
            units_label = 'ticks'
        self._steer_units_label = units_label
        self.get_logger().info(
            '조향 각도 변환: gear_ratio={:.4f}, pulses_per_degree={:.2f} ({})'.format(
                self.steer_gear_ratio,
                self._pulses_per_degree,
                units_label,
            )
        )

        self._mode_byte = struct.pack('<b', self.steer_mode_of_operation)
        self._ctrl_enable = self.steer_control_word_enable & 0xFFFF
        bit5_mask = 0x20 if self.steer_use_bit5 else 0x00
        self._ctrl_enable_lo = self._ctrl_enable | bit5_mask
        self._ctrl_enable_hi = self._ctrl_enable | 0x10 | bit5_mask
        self._toggle_state = False
        self._cob_ids = [
            int(self.steer_motor_rpdo_bases[idx]) + int(node_id)
            for idx, node_id in enumerate(self.steer_motor_ids)
        ]
        self._last_positions = [0.0] * len(self.steer_motor_ids)
        self._have_command = False
        
        # 조향 완료 대기 게이팅 파라미터 로드
        self._steering_gate_enable = bool(self.get_parameter('steering_gate.enable').value)
        self._steering_gate_timeout = float(self.get_parameter('steering_gate.timeout_sec').value)
        self._steering_gate_position_tolerance = int(self.get_parameter('steering_gate.position_tolerance').value)
        self._steering_gate_use_position = bool(self.get_parameter('steering_gate.use_position_feedback').value)
        self._steering_gate_large_error = int(self.get_parameter('steering_gate.large_error_threshold').value)
        if self._steering_gate_timeout < 0.0:
            self._steering_gate_timeout = 0.0
        if self._steering_gate_position_tolerance < 0:
            self._steering_gate_position_tolerance = 500
        if self._steering_gate_large_error < 0:
            self._steering_gate_large_error = 275000
        timeout_str = f"{self._steering_gate_timeout:.2f}s" if self._steering_gate_timeout > 0 else "infinite wait"
        mode_str = "Position" if self._steering_gate_use_position else "StatusWord"
        large_error_deg = self._steering_gate_large_error / self._pulses_per_degree if self._pulses_per_degree > 0 else 0
        self.get_logger().info(
            f"[Steering Gate] enable={self._steering_gate_enable}, timeout={timeout_str}, "
            f"mode={mode_str}, position_tolerance={self._steering_gate_position_tolerance}pulse, "
            f"large_error_threshold={self._steering_gate_large_error}pulse({large_error_deg:.1f}°)"
        )

    def _load_support_params(self) -> None:
        self._support_enable = bool(self.get_parameter('support.enable_startup_tasks').value)

        node_ids = list(self.get_parameter('support.nmt_auto_start.node_ids').value)
        self._nmt_node_ids = [int(node_id) for node_id in node_ids if int(node_id) > 0]

        bus_names_param = list(self.get_parameter('support.nmt_auto_start.bus_names').value)
        if not bus_names_param:
            bus_names_param = ['can0'] * max(1, len(self._nmt_node_ids))
        if len(bus_names_param) < len(self._nmt_node_ids):
            deficit = len(self._nmt_node_ids) - len(bus_names_param)
            self.get_logger().warn(
                'support.nmt_auto_start.bus_names length is insufficient, repeating the last value.'
            )
            last = bus_names_param[-1] if bus_names_param else 'can0'
            bus_names_param.extend([last] * deficit)
        self._nmt_bus_names = [str(name) for name in bus_names_param[:len(self._nmt_node_ids)]]

        self._nmt_enable = bool(self.get_parameter('support.nmt_auto_start.enable').value)
        heartbeat_duration = float(
            self.get_parameter('support.nmt_auto_start.heartbeat_duration_sec').value
        )
        if heartbeat_duration < 0.0:
            self.get_logger().warn('heartbeat_duration_sec is negative, setting to 0.')
            heartbeat_duration = 0.0
        self._nmt_heartbeat_duration = heartbeat_duration
        self._nmt_log_heartbeats = bool(
            self.get_parameter('support.nmt_auto_start.log_heartbeats').value
        )

        self._steer_recovery_enable = bool(
            self.get_parameter('support.steer_recovery.enable').value
        )
        self._steer_recovery_can0_nodes = [
            int(node) for node in list(self.get_parameter('support.steer_recovery.can0_nodes').value)
        ]
        self._steer_recovery_can1_nodes = [
            int(node) for node in list(self.get_parameter('support.steer_recovery.can1_nodes').value)
        ]

    def _run_support_procedures(self) -> None:
        logger = self.get_logger()
        try:
            if self._nmt_enable:
                self._perform_nmt_auto_start()
            if self._support_thread_stop.is_set():
                return
            if self._steer_recovery_enable:
                self._perform_steer_recovery()
        except Exception:  # pragma: no cover - defensive logging
            logger.error('[Support] Startup support task failed:\n' + traceback.format_exc())

    def _perform_nmt_auto_start(self) -> None:
        logger = self.get_logger()
        if can is None:
            logger.warn(
                'python-can module not found, skipping NMT auto start. (pip install python-can).'
            )
            return

        node_config = self._build_nmt_config()
        if not node_config:
            logger.info('[NMT] Auto start node list is empty, skipping NMT auto start.')
            return

        unique_bus_names = sorted(set(node_config.values()))
        logger.info(f"[NMT] Auto start ready: nodes {sorted(node_config.keys())}")

        buses: Dict[str, Any] = {}
        listener_threads: List[threading.Thread] = []
        stop_event = threading.Event()
        node_states: Dict[int, int] = {}
        states_lock = threading.Lock()

        def heartbeat_listener(bus_name: str, bus_obj: Any) -> None:
            label = getattr(bus_obj, 'channel_info', bus_name)
            logger.debug(f"[NMT][{bus_name}] Heartbeat listener 시작 ({label})")
            while not stop_event.is_set() and not self._support_thread_stop.is_set():
                try:
                    msg = bus_obj.recv(timeout=0.1)
                except can.CanError as exc:
                    logger.error(f"[NMT][{bus_name}] Heartbeat 수신 에러: {exc}")
                    break
                if msg is None:
                    continue
                # Heartbeat/Node guarding status frame: 0x700 + node_id (0x701..0x77F)
                if not (0x701 <= msg.arbitration_id <= 0x77F):
                    continue
                if not msg.data:
                    continue
                node_id = msg.arbitration_id - 0x700
                state = msg.data[0]
                with states_lock:
                    previous = node_states.get(node_id)
                    node_states[node_id] = state
                if self._nmt_log_heartbeats or previous != state:
                    state_str = NMT_STATE_MAP.get(state, f'Unknown(0x{state:02X})')
                    logger.info(f"[NMT][{bus_name}] Node {node_id} 상태: {state_str}")
            logger.debug(f"[NMT][{bus_name}] Heartbeat listener 종료")

        try:
            for bus_name in unique_bus_names:
                try:
                    bus = can.interface.Bus(channel=bus_name, bustype='socketcan')
                except can.CanError as exc:
                    logger.error(f"[NMT] CAN bus '{bus_name}' initialization failed: {exc}")
                    return
                buses[bus_name] = bus
                listener = threading.Thread(
                    target=heartbeat_listener,
                    args=(bus_name, bus),
                    name=f'nmt_hb_{bus_name}',
                    daemon=True,
                )
                listener_threads.append(listener)
                listener.start()

            if self._support_thread_stop.wait(self._nmt_heartbeat_duration):
                logger.info('[NMT] initialization heartbeat (shutdown requested)')
                return

            nmt_cob_id = 0x000
            CMD_START_NODE = 0x01
            CMD_START_ALL  = 0x01
            START_RETRIES = 3
            RETRY_INTERVAL = 0.4
            nodes_to_start = 0

            # 1) 우선 각 버스에 "전체 노드 시작"(node_id=0) 브로드캐스트를 1회 전송
            for bus_name, bus in buses.items():
                try:
                    msg_all = can.Message(
                        arbitration_id=nmt_cob_id,
                        data=[CMD_START_ALL, 0x00],  # node_id=0 -> 모든 노드
                        is_extended_id=False,
                    )
                    bus.send(msg_all)
                    logger.info(f"[NMT][{bus_name}] Start(ALL) broadcast sent")
                except can.CanError as exc:
                    logger.error(f"[NMT][{bus_name}] Start(ALL) transmission failed: {exc}")

            for node_id in sorted(node_config.keys()):
                bus_name = node_config[node_id]
                bus = buses.get(bus_name)
                if bus is None:
                    logger.error(f"[NMT] Node {node_id} bus'{bus_name}' cannot be found.")
                    continue
                with states_lock:
                    current_state = node_states.get(node_id)
                if current_state == 0x05:
                    logger.info(f"[NMT] Node {node_id} ({bus_name}): already Operational → skipping")
                    continue
                nodes_to_start += 1
                state_str = NMT_STATE_MAP.get(current_state, 'Unknown')
                logger.info(
                    f"[NMT] Node {node_id} ({bus_name}): current state '{state_str}' → Start command sent"
                )

                # 최대 START_RETRIES 회 재시도하며, 변화 모니터링
                for attempt in range(1, START_RETRIES + 1):
                    try:
                        msg = can.Message(
                            arbitration_id=nmt_cob_id,
                            data=[CMD_START_NODE, node_id],
                            is_extended_id=False,
                        )
                        bus.send(msg)
                        logger.info(f"[NMT] Node {node_id} ({bus_name}) Start command sent (attempt {attempt}/{START_RETRIES})")
                    except can.CanError as exc:
                        logger.error(f"[NMT] Node {node_id} Start command transmission failed: {exc}")
                        # 전송 실패 시 다음 시도로 넘어감
                    # 짧게 대기하며 상태 전이 관찰
                    time_limit = time.monotonic() + RETRY_INTERVAL
                    became_operational = False
                    while time.monotonic() < time_limit and not self._support_thread_stop.is_set():
                        with states_lock:
                            st = node_states.get(node_id)
                        if st == 0x05:
                            became_operational = True
                            break
                        time.sleep(0.05)
                    if became_operational:
                        logger.info(f"[NMT] Node {node_id} ({bus_name}) → Operational (confirmed after attempt {attempt})")
                        break
                else:
                    # 모든 시도 후에도 미전이
                    with states_lock:
                        st = node_states.get(node_id)
                    st_str = NMT_STATE_MAP.get(st, f"Unknown(0x{(st if st is not None else 0):02X})")
                    logger.warn(f"[NMT] Node {node_id} ({bus_name}) still {st_str} (retry limit reached)")
            if nodes_to_start == 0:
                logger.info('[NMT] all nodes are already Operational.')
            else:
                logger.info(f"[NMT] sent Start command to {nodes_to_start} nodes.")

            # 추가로 짧게 상태 수집 시간을 준다
            self._support_thread_stop.wait(0.5)
        finally:
            stop_event.set()
            for listener in listener_threads:
                listener.join(timeout=1.0)
            for bus_name, bus in buses.items():
                try:
                    bus.shutdown()
                    logger.debug(f"[NMT] CAN bus '{bus_name}' shutdown successfully.")
                except (AttributeError, can.CanError):
                    pass

    def _build_nmt_config(self) -> Dict[int, str]:
        config: Dict[int, str] = {}
        for node_id, bus_name in zip(self._nmt_node_ids, self._nmt_bus_names):
            if node_id <= 0:
                continue
            config[int(node_id)] = str(bus_name)
        return config

    def _perform_steer_recovery(self) -> None:
        if not self._steer_recovery_can0_nodes and not self._steer_recovery_can1_nodes:
            self.get_logger().info('[SteerRecovery] No nodes configured for steer recovery, skipping.')
            return
        helper = SteerRecoveryHelper(
            logger=self.get_logger(),
            can0_nodes=self._steer_recovery_can0_nodes,
            can1_nodes=self._steer_recovery_can1_nodes,
            cancel_event=self._support_thread_stop,
        )
        helper.run()
        
        # After recovery, move all wheels to front-facing position (0 degrees) if not shutting down
        if not self._support_thread_stop.is_set():
            self.get_logger().info('[Startup] Moving all wheels to front-facing position (0 deg)...')
            helper.home_to_zero()

    def destroy_node(self) -> bool:  # type: ignore[override]
        self.get_logger().info('[Shutdown] ========== Node shutdown initiated ==========')
        
        # 1. Set shutdown flag to ignore any new cmd_vel commands
        self._is_shutting_down = True
        self.get_logger().info('[Shutdown] Shutdown flag set - ignoring new cmd_vel commands')
        
        if hasattr(self, '_support_thread_stop'):
            self._support_thread_stop.set()
        support_thread = getattr(self, '_support_thread', None)
        if support_thread and support_thread.is_alive():
            support_thread.join(timeout=2.0)

        if getattr(self, 'heartbeat_timer', None):
            self.heartbeat_timer.cancel()
        if getattr(self, 'feedback_watchdog_timer', None):
            self.feedback_watchdog_timer.cancel()

        # 2. Stop all drive motors immediately (RPM=0)
        self._stop_all_drive_motors()
        
        # 3. Move all wheels to front-facing position (0 degrees) before shutdown
        self._home_wheels_on_shutdown()

        # 4. Send NMT Stop to all motors
        self._emergency_stop_all()
        time.sleep(0.1)
        
        self.get_logger().info('[Shutdown] ========== Node shutdown complete ==========')
        return super().destroy_node()

    def _stop_all_drive_motors(self) -> None:
        """
        Stop all drive motors by sending Shutdown command + RPM=0.
        
        IMPORTANT: This method uses direct CAN socket communication instead of ROS2 Publishers
        because during node shutdown, ROS2 Publishers may already be deactivated and unable
        to send messages. This ensures the stop command actually reaches the motor drivers.
        
        This uses Control Word 0x0006 (Shutdown/Ready to Switch On) instead of
        just RPM=0 with Operation Enable (0x000F) to ensure the motors actually stop.
        The Shutdown command transitions the drive from Operation Enable to 
        Ready to Switch On state, which disables the power stage.
        """
        SHUTDOWN_CONTROL_WORD = 0x0006  # Shutdown: transitions to Ready to Switch On state
        DISABLE_VOLTAGE_CONTROL_WORD = 0x0000  # Disable Voltage: completely disables drive
        
        self.get_logger().info('[Shutdown] Stopping all drive motors using direct CAN socket...')
        
        # Group motors by CAN bus
        can0_motors: List[Tuple[int, int, int]] = []  # (idx, node_id, rpdo_base)
        can1_motors: List[Tuple[int, int, int]] = []
        
        for idx, node_id in enumerate(self.drive_motor_ids):
            bus_name = self.drive_motor_bus_names[idx]
            rpdo_base = self.drive_motor_rpdo_bases[idx]
            if bus_name == 'can0':
                can0_motors.append((idx, int(node_id), rpdo_base))
            else:
                can1_motors.append((idx, int(node_id), rpdo_base))
        
        interfaces: Dict[str, CANInterface] = {}
        
        try:
            # Open CAN interfaces directly (bypass ROS2 publishers)
            if can0_motors:
                try:
                    interfaces['can0'] = CANInterface('can0', self.get_logger())
                except CANError as exc:
                    self.get_logger().error(f'[Shutdown] Failed to open can0: {exc}')
            if can1_motors:
                try:
                    interfaces['can1'] = CANInterface('can1', self.get_logger())
                except CANError as exc:
                    self.get_logger().error(f'[Shutdown] Failed to open can1: {exc}')
            
            # Build RPDO payload for stop command
            def build_drive_rpdo(control_word: int, mode: int, pps: int) -> bytes:
                payload = struct.pack('<H', control_word & 0xFFFF)
                payload += struct.pack('<b', int(mode))
                payload += struct.pack('<i', int(pps))
                if len(payload) < 8:
                    payload += b'\x00' * (8 - len(payload))
                return payload
            
            # Step 1: Send Shutdown command (0x0006) with PPS=0 to all motors
            self.get_logger().info('[Shutdown] Step 1: Sending Shutdown (0x0006) + PPS=0...')
            shutdown_payload = build_drive_rpdo(SHUTDOWN_CONTROL_WORD, self.drive_mode_of_operation, 0)
            
            for bus_name, motors in [('can0', can0_motors), ('can1', can1_motors)]:
                interface = interfaces.get(bus_name)
                if not interface:
                    continue
                for idx, node_id, rpdo_base in motors:
                    cob_id = rpdo_base + node_id
                    interface.send(cob_id, shutdown_payload)
                    self.get_logger().info(f'[Shutdown] Drive Node {node_id} ({bus_name}): Shutdown sent to COB-ID 0x{cob_id:03X}')
            
            time.sleep(0.1)
            
            # Step 2: Send Disable Voltage (0x0000) with PPS=0 to all motors
            self.get_logger().info('[Shutdown] Step 2: Sending Disable Voltage (0x0000) + PPS=0...')
            disable_payload = build_drive_rpdo(DISABLE_VOLTAGE_CONTROL_WORD, self.drive_mode_of_operation, 0)
            
            for bus_name, motors in [('can0', can0_motors), ('can1', can1_motors)]:
                interface = interfaces.get(bus_name)
                if not interface:
                    continue
                for idx, node_id, rpdo_base in motors:
                    cob_id = rpdo_base + node_id
                    interface.send(cob_id, disable_payload)
                    self.get_logger().info(f'[Shutdown] Drive Node {node_id} ({bus_name}): Disable Voltage sent to COB-ID 0x{cob_id:03X}')
            
            time.sleep(0.1)
            self.get_logger().info('[Shutdown] All drive motors commanded to stop via direct CAN')
            
        except Exception as exc:
            self.get_logger().error(f'[Shutdown] Failed to stop drive motors: {exc}')
        finally:
            # Close CAN interfaces
            for interface in interfaces.values():
                try:
                    interface.close()
                except Exception:
                    pass

    def _home_wheels_on_shutdown(self) -> None:
        """Move all steer motors to 0 degrees (front-facing) before shutdown."""
        self.get_logger().info('[Shutdown] Moving all wheels to front-facing position (0 deg)...')
        
        # Create a new cancel event (not the one used by support thread)
        shutdown_cancel = threading.Event()
        
        try:
            helper = SteerRecoveryHelper(
                logger=self.get_logger(),
                can0_nodes=self._steer_recovery_can0_nodes,
                can1_nodes=self._steer_recovery_can1_nodes,
                cancel_event=shutdown_cancel,
            )
            success = helper.home_to_zero()
            if success:
                self.get_logger().info('[Shutdown] All wheels moved to front-facing position')
            else:
                self.get_logger().warning('[Shutdown] Some wheels may not have been homed')
        except Exception as exc:
            self.get_logger().error(f'[Shutdown] Failed to home wheels: {exc}')

    def _can_feedback_callback(self, frame: Frame) -> None:
        self.last_can_feedback_time = self.get_clock().now()
        if self._is_feedback_watchdog_tripped:
            self.get_logger().info('CAN feedback reception resumed. Command processing normalized.')
            self._is_feedback_watchdog_tripped = False

        # [2025-11-28] TPDO2 파싱: Position 또는 Status Word 기반 Target Reached 판단
        cob_id = frame.id
        if cob_id in self._steer_tpdo2_cob_ids:
            node_id = self._steer_tpdo2_cob_ids[cob_id]
            prev_state = self._steer_target_reached.get(node_id, False)
            was_all_reached = self._all_steering_reached()
            
            if self._steering_gate_use_position:
                # Position 기반 판단: TPDO2에서 Position Actual Value (4바이트, signed int32)
                if len(frame.data) >= 4:
                    actual_position = struct.unpack('<i', bytes(frame.data[:4]))[0]
                    self._steer_actual_position[node_id] = actual_position
                    target_position = self._steer_target_position.get(node_id, 0)
                    position_error = abs(actual_position - target_position)
                    target_reached = position_error <= self._steering_gate_position_tolerance
                    self._steer_target_reached[node_id] = target_reached
                    
                    # 상태 변화 시 로그
                    if target_reached != prev_state:
                        self.get_logger().debug(
                            f"[steering completion] Node {node_id}: Reached={target_reached} "
                            f"(actual={actual_position}, target={target_position}, err={position_error})"
                        )
            else:
                # Status Word 기반 판단: Bit 10 (Target Reached)
                if len(frame.data) >= 2:
                    status_word = struct.unpack('<H', bytes(frame.data[:2]))[0]
                    target_reached = bool(status_word & (1 << STATUS_WORD_TARGET_REACHED_BIT))
                    self._steer_target_reached[node_id] = target_reached
                    
                    if target_reached != prev_state:
                        self.get_logger().debug(
                            f"[steering completion] Node {node_id}: Target Reached = {target_reached} "
                            f"(StatusWord: 0x{status_word:04X})"
                        )
            
            # 모든 조향 완료 전환 감지 (False → True)
            if not was_all_reached and self._all_steering_reached():
                self.get_logger().info(
                    f"[steering completion] all steering motors aligned - driving enabled | "
                    f"status: [{self._get_steering_status_summary()}]"
                )

    def _all_steering_reached(self) -> bool:
        # 게이팅 비활성화 시 항상 True
        if not self._steering_gate_enable:
            return True
        
        # 첫 번째 조향 명령 수신 전에는 구동 허용
        if not self._steering_gate_active:
            return True
        
        # [2025-11-28] 큰 조향 변화가 없으면 구동 허용 (정상적인 경로 추적)
        if not self._steering_waiting_for_large_change:
            return True
        
        # Position 기반 모드: 저장된 actual/target position을 직접 비교
        # (TPDO2가 이벤트 트리거 방식이라 Position 변화가 없으면 전송 안 됨)
        if self._steering_gate_use_position:
            all_reached = True
            for node_id in self.steer_motor_ids:
                actual = self._steer_actual_position.get(node_id, 0)
                target = self._steer_target_position.get(node_id, 0)
                if abs(actual - target) > self._steering_gate_position_tolerance:
                    all_reached = False
                    break
            if all_reached:
                # 조향 완료 → 대기 모드 해제
                self._steering_waiting_for_large_change = False
                return True
        else:
            # Status Word 기반 모드: TPDO2 콜백에서 업데이트된 상태 확인
            all_reached = all(
                self._steer_target_reached.get(node_id, False)
                for node_id in self.steer_motor_ids
            )
            if all_reached:
                # 조향 완료 → 대기 모드 해제
                self._steering_waiting_for_large_change = False
                return True
        
        # 타임아웃 확인: 조향 명령 후 일정 시간이 지나면 구동 허용
        if self._steering_gate_timeout > 0.0 and self._last_steer_command_time is not None:
            elapsed = (self.get_clock().now() - self._last_steer_command_time).nanoseconds / 1e9
            if elapsed >= self._steering_gate_timeout:
                # 타임아웃 → 대기 모드 해제
                self._steering_waiting_for_large_change = False
                return True
        
        return False

    def _get_steering_status_summary(self) -> str:
        return ', '.join(
            f"{node_id}:{'✓' if self._steer_target_reached.get(node_id, False) else '✗'}"
            for node_id in self.steer_motor_ids
        )

    def _feedback_watchdog_callback(self) -> None:
        elapsed = (self.get_clock().now() - self.last_can_feedback_time).nanoseconds / 1e9
        if elapsed > self.can_feedback_timeout:
            if not self._is_feedback_watchdog_tripped:
                self.get_logger().error(
                    f"CAN feedback is missing for more than {self.can_feedback_timeout:.2f}s. Initiating emergency stop."
                )
                self._command_robot_stop()
                self._is_feedback_watchdog_tripped = True

    def _command_robot_stop(self) -> None:
        zero_rpms = [0.0] * len(self.drive_motor_ids)
        self._send_drive_commands(zero_rpms)
        self.get_logger().info('Drive motors stopped (RPM=0).')

    def _send_sync_heartbeat(self) -> None:
        sync_msg = Frame()
        sync_msg.id = 0x080
        sync_msg.is_extended = False
        sync_msg.dlc = 0
        try:
            self.can0_pub.publish(sync_msg)
            self.can1_pub.publish(sync_msg)
            self.get_logger().debug('SYNC heartbeat sent on both CAN buses')
        except Exception as exc:  # pragma: no cover - publish failures unexpected
            self.get_logger().error(f'SYNC heartbeat transmission failed: {exc}')

    def _emergency_stop_all(self) -> None:
        self.get_logger().fatal('All motors are being stopped with NMT Stop command.')
        nmt_cob_id = 0x000
        cmd_stop_node = 0x02
        for node_id, bus_name in self._all_nodes_config.items():
            stop_msg = Frame()
            stop_msg.id = nmt_cob_id
            stop_msg.is_extended = False
            stop_msg.dlc = 2
            stop_msg.data = [cmd_stop_node, int(node_id) & 0xFF, 0, 0, 0, 0, 0, 0]
            try:
                if bus_name == 'can0':
                    self.can0_pub.publish(stop_msg)
                elif bus_name == 'can1':
                    self.can1_pub.publish(stop_msg)
                else:
                    self.get_logger().warn(
                        f"Unknown bus '{bus_name}' (Node {node_id}). Sending to can0."
                    )
                    self.can0_pub.publish(stop_msg)
                self.get_logger().info(f' -> Node {node_id} ({bus_name}) NMT Stop sent')
            except Exception as exc:  # pragma: no cover - publish failures unexpected
                self.get_logger().error(
                    f" -> Node {node_id} ({bus_name}) NMT Stop transmission failed: {exc}"
                )
            time.sleep(0.01)

    def cmd_vel_callback(self, msg: Twist) -> None:
        if self._is_shutting_down:
            # Ignore cmd_vel during shutdown to prevent drive motors from moving
            return
            
        if self._is_feedback_watchdog_tripped:
            self.get_logger().warn(
                'CAN feedback watchdog triggered. Ignoring /cmd_vel commands.',
                throttle_duration_sec=2.0,
            )
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_msg_time).nanoseconds / 1e9
        self.last_msg_time = current_time

        # 통신 지연으로 인해 dt가 비정상적으로 커졌을 때 오도메트리 점프 방지
        if dt > 0.1:
            dt = 0.02  # 기본 제어 주기(50Hz)로 리셋하여 적분 폭주 방지

        # 1. 목표 조향 각도 및 주행 속도 계산
        steer_angles_raw, wheel_rad_s, debug_data = self.calculate_swerve_outputs(
            msg.linear.x, msg.linear.y, msg.angular.z, dt
        )
        
        # 2. 조향 각도에 오프셋 적용
        steer_angles_post = [
            angle + self._steer_offset_rad[idx]
            for idx, angle in enumerate(steer_angles_raw)
        ]
        
        # 3. 조향 각도 오차 계산 (현재 각도와 목표 각도의 차이)
        angle_errors = [
            abs(target - current) 
            for target, current in zip(steer_angles_post, self._last_positions)
        ]
        max_angle_error = max(angle_errors) if angle_errors else 0.0
        max_angle_error_deg = math.degrees(max_angle_error)
        
        # 4. 주행 속도 스케일 계산 (각도 오차 기반)
        drive_speed_scale = self._calculate_drive_speed_scale(max_angle_error)
        
        # 5. 주행 속도 계산 (rad/s → RPM)
        wheel_rpms = [(rad_s * 60.0) / (2.0 * math.pi) for rad_s in wheel_rad_s]
        
        # 6. 주행 속도에 스케일 적용 (협조 제어)
        wheel_rpms_scaled = [rpm * drive_speed_scale for rpm in wheel_rpms]
        
        # 7. 모터별 방향 반전 적용
        for idx in range(min(len(wheel_rpms_scaled), len(self._drive_invert_flags))):
            if self._drive_invert_flags[idx]:
                wheel_rpms_scaled[idx] = -wheel_rpms_scaled[idx]

       # 조향 게이트 조건 미리 판단
        is_drive_enabled = self._all_steering_reached() or max_angle_error_deg <= 30.0

        # calculate_swerve_outputs()가 self.prev_vx/vy/omega 에 필터된 값을 저장함
        actual_vx = self.prev_vx if is_drive_enabled else 0.0
        actual_vy = self.prev_vy if is_drive_enabled else 0.0
        actual_omega = self.prev_omega if is_drive_enabled else 0.0
        actual_wheel_rpms = wheel_rpms_scaled if is_drive_enabled else [0.0] * len(wheel_rpms_scaled)

        # 8. 주행 속도 토픽 발행 (실제 인가되는 속도 기준)
        drive_msg = Float32MultiArray()
        drive_msg.data = [float(rpm) for rpm in actual_wheel_rpms]
        self.drive_rpm_pub.publish(drive_msg)

        # 9. 조향 각도 토픽 발행
        steer_msg = Float32MultiArray()
        steer_msg.data = steer_angles_post
        self.steer_angle_pub.publish(steer_msg)
        
        # 10. JointState 발행 (바퀴 누적 회전)
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = self.joint_names

        for i in range(4):
            rad_s = actual_wheel_rpms[i] * (2.0 * math.pi / 60.0)
            self.wheel_positions[i] += rad_s * dt

        joint_msg.position = steer_angles_post + self.wheel_positions
        self.joint_state_pub.publish(joint_msg)
        
        # 11. Odometry 계산 및 발행 (실제 움직인 속도 actual_vx 기반으로 적분)
        delta_x = (actual_vx * math.cos(self.robot_theta) - actual_vy * math.sin(self.robot_theta)) * dt
        delta_y = (actual_vx * math.sin(self.robot_theta) + actual_vy * math.cos(self.robot_theta)) * dt
        delta_theta = actual_omega * dt

        self.robot_x += delta_x
        self.robot_y += delta_y
        self.robot_theta += delta_theta

        odom_msg = Odometry()
        odom_msg.header.stamp = joint_msg.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.robot_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.robot_theta / 2.0)

        odom_msg.twist.twist.linear.x = actual_vx
        odom_msg.twist.twist.linear.y = actual_vy
        odom_msg.twist.twist.angular.z = actual_omega

        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[35] = 0.05
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[7] = 0.01
        odom_msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom_msg)

        # 12. 조향 모터에 CAN 명령 전송
        self._last_positions = steer_angles_post.copy()
        self._have_command = True
        self._send_setpoints(self._last_positions, log_info=True)

        # 13. 주행 모터에 CAN 명령 전송 (게이트 조건에 따라 전송)
        self._send_drive_commands(actual_wheel_rpms)
        
        if is_drive_enabled:
            if not self._all_steering_reached():
                self.get_logger().info(
                    f"[Waiting for steering completion] angle error: {max_angle_error_deg:.1f}° <= 30° scale: {drive_speed_scale*100:.0f}% | state: [{self._get_steering_status_summary()}]",
                    throttle_duration_sec=0.5
                )
        else:
            self.get_logger().info(
                f"[Waiting for steering completion]  Waiting for steering alignment - drive stop | state: [{self._get_steering_status_summary()}]",
                throttle_duration_sec=0.3
            )

        # 14. 이전 각도 저장 (다음 계산에 사용)
        self.prev_angles_raw = steer_angles_raw.copy()

        # 15. 디버그 로깅
        if drive_speed_scale < 0.95:
            error_details = ', '.join([
                f"{self._module_labels[i]}:{math.degrees(err):.1f}°"
                for i, err in enumerate(angle_errors)
            ])
            self.get_logger().info(
                f"Coordinated Control: {math.degrees(max_angle_error):.1f}° → "
                f"Drive Speed: {drive_speed_scale*100:.0f}% | Error Details: [{error_details}]",
                throttle_duration_sec=0.5
            )

        self.get_logger().debug(
            "Published RPMs: {} angles(rad): {}".format(
                [f'{rpm:.1f}' for rpm in actual_wheel_rpms],
                [f'{a:.3f}' for a in steer_angles_post],
            )
        )
        
        if self._debug_enabled and debug_data is not None:
            self._log_debug_outputs(msg, dt, debug_data, steer_angles_post, actual_wheel_rpms)
            

    def manual_steer_override_callback(self, msg: Float32MultiArray) -> None:
        if self._is_shutting_down:
            return

        if self._is_feedback_watchdog_tripped:
            self.get_logger().warn(
                'CAN feedback watchdog triggered. Ignoring /manual_steer_override.',
                throttle_duration_sec=2.0,
            )
            return

        expected = len(self.steer_motor_ids)
        if len(msg.data) != expected:
            self.get_logger().warn(
                f'/manual_steer_override length must be {expected}, but got {len(msg.data)}'
            )
            return

        steer_angles_raw = [float(v) for v in msg.data[:expected]]
        steer_angles_post = [
            angle + self._steer_offset_rad[idx]
            for idx, angle in enumerate(steer_angles_raw)
        ]

        steer_msg = Float32MultiArray()
        steer_msg.data = steer_angles_post
        self.steer_angle_pub.publish(steer_msg)

        drive_msg = Float32MultiArray()
        drive_msg.data = [0.0] * len(self.drive_motor_ids)
        self.drive_rpm_pub.publish(drive_msg)

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_omega = 0.0
        self._last_positions = steer_angles_post.copy()
        self._have_command = True
        self._send_setpoints(self._last_positions, log_info=True)

        self._send_drive_commands([0.0] * len(self.drive_motor_ids))
        self.prev_angles_raw = steer_angles_raw.copy()

        deg_list = [round(math.degrees(v), 1) for v in steer_angles_raw]
        self.get_logger().info(f'manual_steer_override applied (deg): {deg_list}')

    def _rpm_to_pps(self, rpm: float) -> int:
        motor_rpm = rpm * DRIVE_GEAR_RATIO
        return int((motor_rpm / 60.0) * self.TARGET_VEL_PPR)

    def _send_drive_commands(self, rpms: List[float], control_word: Optional[int] = None) -> None:
        rpms_local = list(rpms)
        if len(rpms_local) < len(self.drive_motor_ids):
            rpms_local.extend([0.0] * (len(self.drive_motor_ids) - len(rpms_local)))
        
        # Use provided control_word or default to self.drive_control_word
        cw_to_use = control_word if control_word is not None else self.drive_control_word
        
        frame_logs: List[str] = []
        for idx, (node_id, rpm_cmd) in enumerate(zip(self.drive_motor_ids, rpms_local)):
            corrected_rpm = rpm_cmd * self.motor_directions[idx]
            pps_value = self._rpm_to_pps(corrected_rpm)
            frame_logs.append(
                self._send_drive_frame(
                    idx,
                    int(node_id),
                    cw_to_use,
                    self.drive_mode_of_operation,
                    pps_value,
                )
            )
        if frame_logs:
            self.get_logger().info('RPDO publish: ' + '; '.join(frame_logs))

    def _send_drive_frame(self, idx: int, node_id: int, control_word: int, mode: int, pps_value: int) -> str:
        cob_base = self.drive_motor_rpdo_bases[idx]
        cob_id = int(cob_base) + int(node_id)

        payload = struct.pack('<H', control_word & 0xFFFF)
        payload += struct.pack('<b', int(mode))
        payload += struct.pack('<i', int(pps_value))
        if len(payload) < 8:
            payload += b'\x00' * (8 - len(payload))

        frame = Frame()
        frame.id = cob_id
        frame.is_extended = False
        frame.dlc = len(payload)
        frame.data = list(payload)

        bus_name = self.drive_motor_bus_names[idx]
        target_bus = bus_name
        if bus_name == 'can1':
            self.can1_pub.publish(frame)
        elif bus_name == 'can0':
            self.can0_pub.publish(frame)
        else:
            self.get_logger().warn(f"unknown bus '{bus_name}' (Node {node_id}). can0 publish fallback.")
            target_bus = 'can0'
            self.can0_pub.publish(frame)

        return (
            f"Node {node_id} -> COB-ID 0x{cob_id:03X} ({target_bus}) "
            f"CW 0x{control_word & 0xFFFF:04X} MODE {mode} PPS {pps_value}"
        )

    def _run_drive_init_step(self) -> None:
        if self._init_sequence_step >= len(self._init_sequence):
            if self._init_timer:
                self._init_timer.cancel()
                self._init_timer = None
            self.get_logger().info('Initialization RPDO Enable sequence completed.')
            return
        control_word = self._init_sequence[self._init_sequence_step]
        self._init_sequence_step += 1

        logs = []
        for idx, node_id in enumerate(self.drive_motor_ids):
            logs.append(
                self._send_drive_frame(
                    idx,
                    int(node_id),
                    control_word,
                    self.drive_mode_of_operation,
                    0,
                )
            )
        if logs:
            self.get_logger().info(f"[INIT RPDO] CW 0x{control_word & 0xFFFF:04X}: " + '; '.join(logs))

    def _angle_to_units(self, angle_rad: float, node_id: int) -> int:
        angle_deg = math.degrees(angle_rad)
        pulses = angle_deg * self._pulses_per_degree
        if node_id in self.steer_invert_ids:
            pulses = -pulses
        pulses = max(-2**31, min(2**31 - 1, round(pulses)))
        return int(pulses)

    # 조향-주행 협조 제어 파라미터 계산
    def _calculate_drive_speed_scale(self, angle_error_rad: float) -> float:
        if not self._coord_control_enable:
            return 1.0  # 비활성화 시 항상 full speed
        
        if angle_error_rad <= self._coord_full_speed_error:
            return 1.0
        elif angle_error_rad >= self._coord_zero_speed_error:
            return self._coord_min_speed
        else:
            normalized = (angle_error_rad - self._coord_full_speed_error) / \
                        (self._coord_zero_speed_error - self._coord_full_speed_error)
            
            if self._coord_use_smooth:
                # S-curve
                smooth = 1.0 - (3.0 * normalized**2 - 2.0 * normalized**3)
            else:
                # Linear
                smooth = 1.0 - normalized
            
            return self._coord_min_speed + (1.0 - self._coord_min_speed) * smooth

    def _initialize_drive_rpdo_sync(self) -> None:
        self.get_logger().info('Drive motor RPDO1 SYNC synchronization sequence started.')
        
        RPDO1_COMM_PARAM_IDX = 0x1400  # RPDO1 통신 파라미터
        TRANSMISSION_TYPE_SUBIDX = 0x02  # Transmission Type sub-index
        SYNC_TRANSMISSION_TYPE = 0x01  # SYNC 동기식 (0x01 = 매 SYNC마다 업데이트)
        
        for idx, node_id in enumerate(self.drive_motor_ids):
            bus_name = self.drive_motor_bus_names[idx]
            try:
                self._sdo_download(node_id, bus_name, RPDO1_COMM_PARAM_IDX, TRANSMISSION_TYPE_SUBIDX, SYNC_TRANSMISSION_TYPE, 1)
                self.get_logger().info(f"Drive Node {node_id}: RPDO1 Transmission Type SYNC (0x01) setting complete")
                time.sleep(0.05)  # 짧은 딜레이
            except Exception as exc:
                self.get_logger().error(f"Drive Node {node_id} RPDO SYNC setting failed: {exc}")
        
        self.get_logger().info('Drive motor RPDO1 SYNC synchronization sequence completed.')

    def _configure_steer_tpdo2_mapping(self) -> None:
        self.get_logger().info('[TPDO2] steering motor TPDO2 Status Word mapping start...')
        
        TPDO2_TRANSMISSION_TYPE_SUBIDX = 0x02
        TPDO2_EVENT_TRIGGER = 0xFF  # 이벤트 트리거 (값 변경 시 즉시 전송)
        
        for idx, node_id in enumerate(self.steer_motor_ids):
            bus_name = self.steer_motor_bus_names[idx]
            try:
                # 1. TPDO2 매핑 비활성화 (매핑 개수 = 0)
                self._sdo_download(node_id, bus_name, OD_TPDO2_MAPPING, 0x00, 0, 1)
                time.sleep(0.02)
                
                # 2. Status Word (0x6041:00, 16비트) 매핑
                # 매핑 값 = (index << 16) | (subindex << 8) | bit_length
                # 0x60410010 = 0x6041 << 16 | 0x00 << 8 | 0x10 (16비트)
                self._sdo_download(node_id, bus_name, OD_TPDO2_MAPPING, 0x01, STATUS_WORD_MAPPING_VALUE, 4)
                time.sleep(0.02)
                
                # 3. TPDO2 매핑 활성화 (매핑 개수 = 1)
                self._sdo_download(node_id, bus_name, OD_TPDO2_MAPPING, 0x00, 1, 1)
                time.sleep(0.02)
                
                # 4. TPDO2 Transmission Type을 이벤트 트리거(0xFF)로 설정
                # Status Word가 변경될 때마다 즉시 전송
                self._sdo_download(node_id, bus_name, OD_TPDO2_COMM, TPDO2_TRANSMISSION_TYPE_SUBIDX, TPDO2_EVENT_TRIGGER, 1)
                time.sleep(0.02)
                
                tpdo2_cob_id = TPDO2_COB_ID_BASE + node_id
                self.get_logger().info(
                    f"[TPDO2] Node {node_id} ({bus_name}): Status Word mapping complete - TPDO2 will transmit on change (COB-ID: 0x{tpdo2_cob_id:03X})"
                    f"(COB-ID: 0x{tpdo2_cob_id:03X}, event-triggered)"
                )
                
            except Exception as exc:
                self.get_logger().error(
                    f"[TPDO2] Node {node_id} TPDO2 Status Word mapping failed: {exc}"
                )
        
        self.get_logger().info('[TPDO2] steering motor TPDO2 Status Word mapping completed.')

    def _initialize_steer_motors(self) -> None:
        self.get_logger().info('steering motor initialization: mode/profile setting and Servo ON sequence execution')
        
        # RPDO SYNC 동기화 설정 상수
        RPDO1_COMM_PARAM_IDX = 0x1400  # RPDO1 통신 파라미터
        TRANSMISSION_TYPE_SUBIDX = 0x02  # Transmission Type sub-index
        SYNC_TRANSMISSION_TYPE = 0x01  # SYNC 동기식 (0x01 = 매 SYNC마다 업데이트)
        
        # TPDO2 Position 기반 판단 시에는 매핑 변경 불필요
        # 드라이버가 이미 Position Actual Value를 TPDO2로 전송하고 있음
        # Status Word 기반 판단 시에만 매핑 설정 시도
        if not self._steering_gate_use_position:
            self._configure_steer_tpdo2_mapping()
        
        for idx, node_id in enumerate(self.steer_motor_ids):
            bus_name = self.steer_motor_bus_names[idx]
            try:
                # RPDO1 Transmission Type을 SYNC 동기식(0x01)으로 설정
                # 이렇게 하면 SYNC 메시지(0x080) 수신 시 모든 모터가 동시에 명령 실행
                self._sdo_download(node_id, bus_name, RPDO1_COMM_PARAM_IDX, TRANSMISSION_TYPE_SUBIDX, SYNC_TRANSMISSION_TYPE, 1)
                self.get_logger().info(f"Node {node_id}: RPDO1 Transmission Type SYNC synchronization (0x01) setting complete")
                time.sleep(self.servo_on_sleep)
                
                self._sdo_download(node_id, bus_name, OD_MODES_OF_OPERATION, 0, self.steer_mode_of_operation, 1)
                time.sleep(self.servo_on_sleep)
                self._sdo_download(node_id, bus_name, OD_PROFILE_VELOCITY, 0, self.profile_velocity, 4)
                self._sdo_download(node_id, bus_name, OD_PROFILE_ACCELERATION, 0, self.profile_accel, 4)
                self._sdo_download(node_id, bus_name, OD_PROFILE_DECELERATION, 0, self.profile_decel, 4)
                time.sleep(self.servo_on_sleep)
                for ctrl_word in (0x0006, 0x0007, self._ctrl_enable_lo & 0xFFFF):
                    self._sdo_download(node_id, bus_name, OD_CONTROL_WORD, 0, ctrl_word, 2)
                    time.sleep(self.servo_on_sleep)
                cob_id = self._cob_ids[idx]
                zero_pos = self._angle_to_units(0.0, node_id)
                self._publish_to_bus(bus_name, self._frame(cob_id, self._ctrl_enable_hi, zero_pos))
                if self.trigger_pulse_delay > 0.0:
                    time.sleep(self.trigger_pulse_delay)
                self._publish_to_bus(bus_name, self._frame(cob_id, self._ctrl_enable_lo, zero_pos))
                self.get_logger().info(f"Node {node_id}: position command sent (bus={bus_name})")
            except Exception as exc:
                self.get_logger().error(f"Node {node_id} initialization failed: {exc}")

    def _sdo_download(self, node_id: int, bus_name: str, index: int, subindex: int, value: int, size: int) -> None:
        if size == 1:
            cs = 0x2F
            data_bytes = struct.pack('<b', int(value))
        elif size == 2:
            cs = 0x2B
            data_bytes = struct.pack('<H', int(value) & 0xFFFF)
        elif size == 4:
            cs = 0x23
            data_bytes = struct.pack('<i', int(value))
        else:
            self.get_logger().error(f"SDO size {size}B is not supported. index=0x{index:04X}")
            return

        payload = [cs, index & 0xFF, (index >> 8) & 0xFF, subindex]
        payload.extend(data_bytes)
        while len(payload) < 8:
            payload.append(0)

        frame = Frame()
        frame.id = 0x600 + int(node_id)
        frame.is_extended = False
        frame.dlc = 8
        frame.data = payload

        self._publish_to_bus(bus_name, frame)

    def _frame(self, cob_id: int, ctrl_word: int, position: int) -> Frame:
        msg = Frame()
        msg.id = int(cob_id)
        msg.is_extended = False
        payload = self._build_payload(ctrl_word, position)
        msg.dlc = len(payload)
        msg.data = payload
        return msg

    def _build_payload(self, ctrl_word: int, position: int) -> List[int]:
        payload = struct.pack('<H', ctrl_word & 0xFFFF)
        payload += self._mode_byte
        payload += struct.pack('<i', int(position))
        if len(payload) < 8:
            payload += b'\x00' * (8 - len(payload))
        return list(payload)

    def _publish_to_bus(self, bus_name: str, frame: Frame) -> None:
        if bus_name == 'can1':
            self.can1_pub.publish(frame)
        elif bus_name == 'can0':
            self.can0_pub.publish(frame)
        else:
            self.get_logger().warn(f"unknown bus '{bus_name}' → can0 usage fallback.")
            self.can0_pub.publish(frame)

    def _send_frames(self, cob_id: int, bus_name: str, pos: int) -> None:
        if self.steer_trigger_mode == 'always':
            self._publish_to_bus(bus_name, self._frame(cob_id, self._ctrl_enable_hi, pos))
        elif self.steer_trigger_mode == 'toggle':
            ctrl = self._ctrl_enable_hi if self._toggle_state else self._ctrl_enable_lo
            self._publish_to_bus(bus_name, self._frame(cob_id, ctrl, pos))
            self._toggle_state = not self._toggle_state
        else:
            self._publish_to_bus(bus_name, self._frame(cob_id, self._ctrl_enable_hi, pos))
            if self.trigger_pulse_delay > 0.0:
                time.sleep(self.trigger_pulse_delay)
            self._publish_to_bus(bus_name, self._frame(cob_id, self._ctrl_enable_lo, pos))

    def _send_setpoints(self, angles: List[float], log_info: bool = True) -> None:
        # 조향 명령 전송 시간 기록 및 게이팅 활성화
        self._last_steer_command_time = self.get_clock().now()
        self._steering_gate_active = True
        
        # 큰 조향 오차 감지: "현재 위치와 목표 위치의 차이"가 큰 경우 대기
        # (목표 변화량이 아닌, 실제 조향해야 할 양을 기준으로 판단)
        has_large_error = False
        for idx, angle in enumerate(angles):
            if idx >= len(self.steer_motor_ids):
                break
            node_id = self.steer_motor_ids[idx]
            position_units = self._angle_to_units(angle, node_id)
            actual_position = self._steer_actual_position.get(node_id, 0)
            error = abs(position_units - actual_position)
            if error > self._steering_gate_large_error:
                has_large_error = True
                break
        
        # 큰 오차가 있을 때만 대기 모드 활성화
        if has_large_error:
            self._steering_waiting_for_large_change = True
        
        frame_logs = []
        for idx, angle in enumerate(angles):
            if idx >= len(self.steer_motor_ids):
                break
            node_id = self.steer_motor_ids[idx]
            cob_id = self._cob_ids[idx]
            bus_name = self.steer_motor_bus_names[idx]
            position_units = self._angle_to_units(angle, node_id)
            
            # 목표 위치 저장
            # Position 기반 판단에서 사용 (TPDO2 actual position과 비교)
            prev_target = self._steer_target_position.get(node_id, 0)
            self._steer_target_position[node_id] = position_units
            
            # Position 기반 모드: Target Reached 리셋 안 함 (TPDO2 콜백에서 실시간 판단)
            # Status Word 기반 모드: 목표 위치가 변경되었을 때만 리셋
            if not self._steering_gate_use_position:
                # Status Word 모드: 목표가 변경되었을 때만 리셋
                if abs(position_units - prev_target) > self._steering_gate_position_tolerance:
                    self._steer_target_reached[node_id] = False
            
            self._send_frames(cob_id, bus_name, position_units)
            label = self._module_labels[idx] if idx < len(self._module_labels) else f'M{idx}'
            frame_logs.append(
                "Node {node} ({label}) -> COB-ID 0x{cob:03X} ({bus}) POS {pos} angle {rad:.3f} rad / {deg:.1f} deg".format(
                    node=node_id,
                    label=label,
                    cob=cob_id,
                    bus=bus_name,
                    pos=position_units,
                    rad=angle,
                    deg=math.degrees(angle),
                )
            )
        if frame_logs:
            message = 'RPDO steering publication: ' + '; '.join(frame_logs)
            if log_info:
                self.get_logger().info(message)
            else:
                self.get_logger().debug(message)

    def _retrigger_publish(self) -> None:
        if not self._have_command:
            return
        self._send_setpoints(self._last_positions, log_info=False)

    def calculate_swerve_outputs(
        self,
        vx_cmd: float,
        vy_cmd: float,
        omega_cmd: float,
        dt: float,
    ) -> Tuple[List[float], List[float], Optional[Dict[str, object]]]:
        debug: Optional[Dict[str, object]] = {} if self._debug_enabled else None

        vx = max(-self.vmax, min(self.vmax, vx_cmd))
        vy = max(-self.vmax, min(self.vmax, vy_cmd))
        omega = max(-self.wmax, min(self.wmax, omega_cmd))

        if debug is not None:
            debug['input_cmd'] = {'vx': vx_cmd, 'vy': vy_cmd, 'omega': omega_cmd}
            debug['clamped_cmd'] = {'vx': vx, 'vy': vy, 'omega': omega}
            debug['dt'] = dt

        vx_filtered = vx
        vy_filtered = vy
        omega_filtered = omega

        alpha_vx = alpha_vy = alpha_omega = None
        if dt > 0:
            alpha_vx = min(1.0, self.a_max * dt / max(1e-6, abs(vx - self.prev_vx)))
            alpha_vy = min(1.0, self.a_max * dt / max(1e-6, abs(vy - self.prev_vy)))
            alpha_omega = min(1.0, self.alpha_max * dt / max(1e-6, abs(omega - self.prev_omega)))
            vx_filtered = self.prev_vx + alpha_vx * (vx - self.prev_vx)
            vy_filtered = self.prev_vy + alpha_vy * (vy - self.prev_vy)
            omega_filtered = self.prev_omega + alpha_omega * (omega - self.prev_omega)

        vx = vx_filtered
        vy = vy_filtered
        omega = omega_filtered

        if debug is not None:
            debug['smoothed_cmd'] = {
                'vx': vx,
                'vy': vy,
                'omega': omega,
                'alpha_vx': alpha_vx if dt > 0 else None,
                'alpha_vy': alpha_vy if dt > 0 else None,
                'alpha_omega': alpha_omega if dt > 0 else None,
            }

        self.prev_vx, self.prev_vy, self.prev_omega = vx, vy, omega

        vecs: List[Tuple[float, float]] = []
        raw_speeds: List[float] = []
        raw_angles: List[float] = []
        for (x_pos, y_pos) in self._module_positions:
            vx_module = vx - omega * y_pos
            vy_module = vy + omega * x_pos
            vecs.append((vx_module, vy_module))
            raw_speeds.append(math.hypot(vx_module, vy_module))
            raw_angles.append(math.atan2(vy_module, vx_module))

        if debug is not None:
            debug['module_vectors'] = [
                {
                    'module': label,
                    'pos_x': pos[0],
                    'pos_y': pos[1],
                    'vx': comp[0],
                    'vy': comp[1],
                    'raw_speed': speed,
                    'raw_angle': angle,
                }
                for label, pos, comp, speed, angle in zip(
                    self._module_labels,
                    self._module_positions,
                    vecs,
                    raw_speeds,
                    raw_angles,
                )
            ]
        steer_angles_pre: List[float] = []
        wheel_linear_speeds: List[float] = []
        module_decisions = []
        
        for i, (raw_angle, speed) in enumerate(zip(raw_angles, raw_speeds)):
            prev = self.prev_angles_raw[i]

            # 옵션 1: 현재 방향 유지 (raw_angle에 가장 가까운 각도)
            a0, d0, clamped0 = self._nearest_with_meta(raw_angle, prev)
            s0 = speed

            # 옵션 2: 플립 (raw_angle + 180도에 가장 가까운 각도, 속도 반전)
            a1, d1, clamped1 = self._nearest_with_meta(raw_angle + math.pi, prev)
            s1 = -speed

            # 최적 옵션 선택 로직
            # 우선순위 1: 클램핑 회피 - 클램핑되지 않은 옵션 우선 선택
            if clamped0 and not clamped1:
                # 옵션 1만 클램핑됨 -> 옵션 2(플립) 선택
                use_flip = True
                reason = "option1(current_direction)_clamped"
            elif clamped1 and not clamped0:
                # 옵션 2만 클램핑됨 -> 옵션 1 선택
                use_flip = False
                reason = "option2(flip)_clamped"
            else:
                # 우선순위 2: 최소 회전 각도 선택
                # 둘 다 클램핑되지 않았거나, 둘 다 클램핑된 경우
                # 이전 각도에서 변화량이 작은 쪽 선택
                abs_delta0 = abs(d0)
                abs_delta1 = abs(d1)
                
                if self._flip_enabled:
                    # 히스테리시스 적용: 플립 시 약간의 이득이 있어야 플립 선택
                    use_flip = abs_delta1 + self.FLIP_HYST_RAD < abs_delta0
                    reason = f"flip_with_hysteresis(Δ0={abs_delta0:.3f}, Δ1={abs_delta1:.3f})"
                else:
                    # 히스테리시스 없이 단순 최소값 선택
                    use_flip = abs_delta1 < abs_delta0
                    reason = f"min_rotation(Δ0={abs_delta0:.3f}, Δ1={abs_delta1:.3f})"

            # 선택된 옵션 적용
            if use_flip:
                steer_angles_pre.append(a1)
                wheel_linear_speeds.append(s1)
            else:
                steer_angles_pre.append(a0)
                wheel_linear_speeds.append(s0)

            if debug is not None:
                module_decisions.append({
                    'module': self._module_labels[i],
                    'raw_angle': raw_angle,
                    'raw_speed': speed,
                    'prev_angle': prev,
                    'keep_angle': a0,
                    'keep_delta': d0,
                    'keep_speed': s0,
                    'flip_angle': a1,
                    'flip_delta': d1,
                    'flip_speed': s1,
                    'used_flip': use_flip,
                    'final_angle': steer_angles_pre[-1],
                    'final_speed': wheel_linear_speeds[-1],
                })

        self.prev_angles_raw = steer_angles_pre.copy()

        wheel_rad_s = [s / self.r for s in wheel_linear_speeds]
        max_speed = max((abs(w) for w in wheel_rad_s), default=0.0)
        scale = 1.0
        if self.wwheel_rad_s > 0 and max_speed > self.wwheel_rad_s:
            scale = self.wwheel_rad_s / max_speed
            wheel_rad_s = [w * scale for w in wheel_rad_s]

        if debug is not None:
            debug['module_decisions'] = module_decisions
            debug['wheel_rad_s'] = wheel_rad_s.copy()
            debug['speed_scale'] = scale

        return steer_angles_pre, wheel_rad_s, debug

    def _nearest_with_meta(self, raw: float, prev: float) -> Tuple[float, float, bool]:
        two_pi = 2.0 * math.pi
        k = round((prev - raw) / two_pi)
        candidate = raw + k * two_pi

        clamped = False
        if candidate > self.STEER_LIMIT_RAD:
            candidate = self.STEER_LIMIT_RAD
            clamped = True
        elif candidate < -self.STEER_LIMIT_RAD:
            candidate = -self.STEER_LIMIT_RAD
            clamped = True

        return candidate, (candidate - prev), clamped

    def _log_debug_outputs(
        self,
        msg: Twist,
        dt: float,
        debug_data: Dict[str, object],
        steer_angles_post: List[float],
        wheel_rpms: List[float],
    ) -> None:
        self.get_logger().info(
            "[DEBUG] cmd_vel raw: vx={:.3f}, vy={:.3f}, omega={:.3f}, dt={:.3f}s".format(
                debug_data['input_cmd']['vx'],
                debug_data['input_cmd']['vy'],
                debug_data['input_cmd']['omega'],
                dt,
            )
        )
        clamped = debug_data['clamped_cmd']
        self.get_logger().info(
            "[DEBUG] cmd_vel clamped: vx={:.3f}, vy={:.3f}, omega={:.3f}".format(
                clamped['vx'],
                clamped['vy'],
                clamped['omega'],
            )
        )
        smoothed = debug_data.get('smoothed_cmd', {})
        if smoothed:
            self.get_logger().info(
                "[DEBUG] cmd_vel smoothed: vx={:.3f}, vy={:.3f}, omega={:.3f}".format(
                    smoothed['vx'],
                    smoothed['vy'],
                    smoothed['omega'],
                )
            )
        for module in debug_data.get('module_decisions', []):
            self.get_logger().info(
                ("[DEBUG] {} raw_angle={:.1f}deg raw_speed={:.3f}m/s prev={:.1f}deg "
                 "keep={:.1f}deg Δ={:.1f}deg flip={:.1f}deg Δ={:.1f}deg used_flip={} "
                 "final_angle={:.1f}deg final_speed={:.3f}m/s").format(
                    module['module'],
                    math.degrees(module['raw_angle']),
                    module['raw_speed'],
                    math.degrees(module['prev_angle']),
                    math.degrees(module['keep_angle']),
                    math.degrees(module['keep_delta']),
                    math.degrees(module['flip_angle']),
                    math.degrees(module['flip_delta']),
                    module['used_flip'],
                    math.degrees(module['final_angle']),
                    module['final_speed'],
                )
            )
        speed_scale = debug_data.get('speed_scale', 1.0)
        self.get_logger().info(
            "[DEBUG] wheel_rad_s (scale {:.3f}): {}".format(
                speed_scale,
                ', '.join(
                    f"{label}:{value:.3f}"
                    for label, value in zip(self._module_labels, debug_data.get('wheel_rad_s', []))
                ),
            )
        )
        self.get_logger().info(
            "[DEBUG] wheel_rpms final: {}".format(
                ', '.join(f"{label}:{rpm:.1f}" for label, rpm in zip(self._module_labels, wheel_rpms))
            )
        )
        self.get_logger().info(
            "[DEBUG] steer_angles final (rad/deg): {}".format(
                ', '.join(
                    f"{label}:{angle:.3f}rad/{math.degrees(angle):.1f}deg"
                    for label, angle in zip(self._module_labels, steer_angles_post)
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedSwerveController0930()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()