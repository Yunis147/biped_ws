"""A pretend servo bus.

`MockBus` has the same interface as `SerialBus` (transfer / sync_read / send_no_reply) but instead of a
serial port it decodes the real SCS/Feetech packets and emulates ST3215 servos behind them:

  * registers (ID, torque enable, target position/speed/accel, current position, voltage, temperature ...)
  * EEPROM lock flag (ID changes only work after unlocking, like the real servo)
  * the "write 128 to torque-enable = set mid-point" command (position correction)
  * servos move towards their target at the commanded speed cap while torque is on

Used for  (1) `mock:=true` dry runs of the whole stack with no hardware, and (2) the automated tests.
It is NOT a physics simulation (use Gazebo for that); servos just slide to their targets.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from .serial_bus import SerialBusError
from .st3215.registers import Instr, Reg


@dataclass
class _Servo:
    sid: int
    raw: float = 2048.0          # physical encoder position (0..4095), what the magnet sees
    correction: int = 0          # EEPROM position correction: reported = raw - correction
    torque: bool = False
    target: int = 2048
    speed_cap: int = 0           # steps/s, 0 = unlimited
    accel: int = 0
    torque_limit: int = 1000
    lock: int = 1                # EEPROM lock flag (1 = locked)
    voltage_dv: int = 74         # 7.4 V
    temp_c: int = 30
    eeprom: dict = field(default_factory=dict)

    @property
    def reported(self) -> int:
        return int(round(self.raw - self.correction)) % 4096


class MockBus:
    def __init__(self, servo_ids, initial_raw: dict[int, float] | None = None,
                 time_fn: Callable[[], float] = time.monotonic) -> None:
        self.servos = {i: _Servo(i) for i in servo_ids}
        for i, v in (initial_raw or {}).items():
            self.servos[i].raw = float(v)
        self._time = time_fn
        self._t_last = time_fn()
        self._open = False
        self.writes = 0           # number of write packets handled (for tests)
        self.log: list[tuple] = []

    # ---- lifecycle (same as SerialBus) -------------------------------------
    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    # ---- test helpers -------------------------------------------------------
    def move_by_hand(self, sid: int, raw: float) -> None:
        """Pretend a person moved the joint (only possible with torque off)."""
        s = self.servos[sid]
        if s.torque:
            raise RuntimeError("servo is holding torque: cannot move by hand")
        s.raw = float(raw) % 4096

    def reported(self, sid: int) -> int:
        self._advance()
        return self.servos[sid].reported

    # ---- physics-lite ---------------------------------------------------------
    def _advance(self) -> None:
        now = self._time()
        dt = max(0.0, now - self._t_last)
        self._t_last = now
        if dt <= 0:
            return
        for s in self.servos.values():
            if not s.torque:
                continue
            goal_raw = (s.target + s.correction) % 4096
            err = goal_raw - s.raw
            if err > 2048:
                err -= 4096
            elif err < -2048:
                err += 4096
            if s.speed_cap > 0:
                step = s.speed_cap * dt
                err = max(-step, min(step, err))
            s.raw = (s.raw + err) % 4096

    # ---- register file ----------------------------------------------------------
    def _read_reg(self, s: _Servo, reg: int, length: int) -> bytes:
        out = bytearray()
        for a in range(reg, reg + length):
            out.append(self._read_byte(s, a))
        return bytes(out)

    def _read_byte(self, s: _Servo, a: int) -> int:
        pos = s.reported
        table = {
            Reg.FIRMWARE_MAJOR: 3, Reg.FIRMWARE_MINOR: 10, Reg.SERVO_MAJOR: 9, Reg.SERVO_MINOR: 3,
            Reg.ID: s.sid, Reg.TORQUE_ENABLE: int(s.torque), Reg.LOCK_FLAG: s.lock,
            Reg.ACCELERATION: s.accel,
            Reg.TARGET_POS_L: s.target & 0xFF, Reg.TARGET_POS_L + 1: (s.target >> 8) & 0xFF,
            Reg.TARGET_SPEED_L: s.speed_cap & 0xFF, Reg.TARGET_SPEED_L + 1: (s.speed_cap >> 8) & 0xFF,
            Reg.TORQUE_LIMIT_L: s.torque_limit & 0xFF, Reg.TORQUE_LIMIT_L + 1: (s.torque_limit >> 8) & 0xFF,
            Reg.CURRENT_POS_L: pos & 0xFF, Reg.CURRENT_POS_L + 1: (pos >> 8) & 0xFF,
            Reg.CURRENT_SPEED_L: 0, Reg.CURRENT_SPEED_L + 1: 0,
            Reg.CURRENT_LOAD_L: 0, Reg.CURRENT_LOAD_L + 1: 0,
            Reg.CURRENT_VOLTAGE: s.voltage_dv, Reg.CURRENT_TEMP: s.temp_c,
        }
        if a in table:
            return table[a]
        return s.eeprom.get(a, 0)

    def _write_reg(self, s: _Servo, reg: int, data: bytes) -> None:
        self.writes += 1
        i = 0
        while i < len(data):
            a, v = reg + i, data[i]
            if a == Reg.TORQUE_ENABLE:
                if v == 128:                       # "set current position as the mid-point"
                    s.correction = int(round(s.raw)) - 2048
                    s.target = 2048
                    self.log.append(("midpoint", s.sid))
                else:
                    s.torque = bool(v)
            elif a == Reg.ACCELERATION:
                s.accel = v
            elif a == Reg.TARGET_POS_L and i + 1 < len(data):
                s.target = (data[i] | (data[i + 1] << 8)) & 0x0FFF
                i += 1
            elif a == Reg.RUN_TIME_L:
                pass                                  # time register: ignored by the mock
            elif a == Reg.TARGET_SPEED_L and i + 1 < len(data):
                s.speed_cap = data[i] | (data[i + 1] << 8)
                i += 1
            elif a == Reg.TORQUE_LIMIT_L and i + 1 < len(data):
                s.torque_limit = data[i] | (data[i + 1] << 8)
                i += 1
            elif a == Reg.LOCK_FLAG:
                s.lock = v
            elif a == Reg.ID:
                if s.lock != 0:
                    pass                              # EEPROM locked: write ignored, like the real servo
                else:
                    self.servos.pop(s.sid)
                    s.sid = v
                    self.servos[v] = s
                    self.log.append(("id", v))
            else:
                s.eeprom[a] = v
            i += 1

    # ---- packet handling ------------------------------------------------------------
    @staticmethod
    def _parse(packet: bytes):
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            raise SerialBusError("mock: bad packet header")
        sid, length, instr = packet[2], packet[3], packet[4]
        params = packet[5:-1]
        if len(packet) != length + 4:
            raise SerialBusError("mock: bad packet length")
        chk = (~(sid + length + instr + sum(params))) & 0xFF
        if chk != packet[-1]:
            raise SerialBusError("mock: bad packet checksum")
        return sid, instr, bytes(params)

    def transfer(self, packet: bytes, response_data_len: int) -> bytes:
        if not self._open:
            raise SerialBusError("Serial bus is not open")
        self._advance()
        sid, instr, params = self._parse(packet)
        if sid not in self.servos:
            raise SerialBusError(f"Response timeout: expected {6 + response_data_len} bytes, got 0")
        s = self.servos[sid]
        if instr == Instr.PING:
            return b""
        if instr == Instr.READ:
            return self._read_reg(s, params[0], params[1])
        if instr == Instr.WRITE:
            self._write_reg(s, params[0], params[1:])
            return b""
        raise SerialBusError(f"mock: unsupported instruction {instr:#x}")

    def sync_read(self, packet: bytes, servo_ids: list[int], data_len: int) -> dict[int, bytes]:
        if not self._open:
            raise SerialBusError("Serial bus is not open")
        self._advance()
        sid, instr, params = self._parse(packet)
        if instr != Instr.SYNC_READ:
            raise SerialBusError("mock: not a SYNC_READ packet")
        reg, n, ids = params[0], params[1], list(params[2:])
        if n != data_len or ids != list(servo_ids):
            raise SerialBusError("mock: SYNC_READ packet does not match request")
        if any(i not in self.servos for i in ids):
            return {}                                    # real bus: short read -> everything dropped
        return {i: self._read_reg(self.servos[i], reg, n) for i in ids}

    def send_no_reply(self, packet: bytes) -> None:
        if not self._open:
            raise SerialBusError("Serial bus is not open")
        self._advance()
        sid, instr, params = self._parse(packet)
        if instr == Instr.SYNC_WRITE:
            reg, n = params[0], params[1]
            rest = params[2:]
            stride = 1 + n
            for k in range(0, len(rest), stride):
                chunk = rest[k:k + stride]
                if len(chunk) != stride:
                    raise SerialBusError("mock: truncated SYNC_WRITE")
                if chunk[0] in self.servos:
                    self._write_reg(self.servos[chunk[0]], reg, bytes(chunk[1:]))
        elif instr == Instr.WRITE and sid == 0xFE:
            for s in list(self.servos.values()):
                self._write_reg(s, params[0], params[1:])
        else:
            raise SerialBusError(f"mock: unsupported broadcast instruction {instr:#x}")
