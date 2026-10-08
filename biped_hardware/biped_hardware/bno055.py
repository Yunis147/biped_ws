"""BNO055 IMU driver (I2C), adapted from AsterisCrack/BipedRobotJetson hardware/imu/bno055.py (MIT).

Changes from the original: (1) reads the accelerometer INCLUDING gravity (the Gazebo IMU does too, so
/imu looks the same in simulation and on the robot), (2) selectable fusion mode, (3) a MockBNO055.

Uses Adafruit_PureIO.smbus (direct /dev/i2c-N access):  pip install adafruit-pureio
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass
class IMUReading:
    quaternion: tuple = (1.0, 0.0, 0.0, 0.0)     # w, x, y, z
    accel: tuple = (0.0, 0.0, 9.81)              # m/s^2, includes gravity
    gyro: tuple = (0.0, 0.0, 0.0)                # rad/s
    calibration: tuple = (0, 0, 0, 0)            # sys, gyro, accel, mag  (each 0..3, 3 = fully calibrated)


class BNO055:
    _REG_CHIP_ID, _REG_PAGE_ID = 0x00, 0x07
    _REG_ACC = 0x08               # accel(6) mag(6) gyro(6) euler(6) quaternion(8) = 32 bytes
    _REG_CALIB_STAT, _REG_OPR_MODE, _REG_PWR_MODE, _REG_SYS_TRIG = 0x35, 0x3D, 0x3E, 0x3F
    _MODES = {"imuplus": 0x08, "ndof": 0x0C}
    _ACC_SCALE = 1.0 / 100.0                      # m/s^2 per LSB
    _GYRO_SCALE = math.pi / (16.0 * 180.0)        # rad/s per LSB (16 LSB = 1 deg/s)
    _QUAT_SCALE = 1.0 / 16384.0

    def __init__(self, i2c_bus: int = 7, address: int = 0x28, mode: str = "imuplus") -> None:
        if mode not in self._MODES:
            raise ValueError(f"mode must be one of {list(self._MODES)}")
        self.bus_num, self.address, self.mode = i2c_bus, address, mode
        self._smbus = None
        self._last = IMUReading()
        self._n = 0

    def initialize(self) -> None:
        from Adafruit_PureIO import smbus
        self._smbus = smbus.SMBus(self.bus_num)
        chip = self._smbus.read_byte_data(self.address, self._REG_CHIP_ID)
        if chip != 0xA0:
            self._smbus.close()
            self._smbus = None
            raise RuntimeError(f"BNO055 not found at 0x{self.address:02X} on I2C bus {self.bus_num} "
                               f"(chip id 0x{chip:02X}, expected 0xA0)")
        w = self._smbus.write_byte_data
        w(self.address, self._REG_OPR_MODE, 0x00)       # config mode
        time.sleep(0.025)
        w(self.address, self._REG_SYS_TRIG, 0x20)       # software reset
        time.sleep(0.65)
        w(self.address, self._REG_PWR_MODE, 0x00)       # normal power
        time.sleep(0.01)
        w(self.address, self._REG_PAGE_ID, 0x00)
        w(self.address, self._REG_OPR_MODE, self._MODES[self.mode])
        time.sleep(0.025)
        log.info("BNO055 ready on I2C bus %d addr 0x%02X, mode %s", self.bus_num, self.address, self.mode)

    def read(self) -> IMUReading:
        if self._smbus is None:
            return self._last
        try:
            b = self._smbus.read_i2c_block_data(self.address, self._REG_ACC, 32)
            acc = tuple(_s16(b[i], b[i + 1]) * self._ACC_SCALE for i in (0, 2, 4))
            gyr = tuple(_s16(b[i], b[i + 1]) * self._GYRO_SCALE for i in (12, 14, 16))
            q = tuple(_s16(b[i], b[i + 1]) * self._QUAT_SCALE for i in (24, 26, 28, 30))   # w, x, y, z
            self._n = (self._n + 1) % 50
            cal = self._last.calibration
            if self._n == 0:
                c = self._smbus.read_byte_data(self.address, self._REG_CALIB_STAT)
                cal = ((c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3)
            self._last = IMUReading(quaternion=q, accel=acc, gyro=gyr, calibration=cal)
        except Exception as exc:
            log.warning("IMU read error: %s", exc)
        return self._last

    def close(self) -> None:
        if self._smbus is not None:
            try:
                self._smbus.close()
            except Exception:
                pass
            self._smbus = None


class MockBNO055:
    """Stands in for the IMU when there is none (mock:=true): level, at rest, gravity on +z."""
    def initialize(self) -> None:
        pass

    def read(self) -> IMUReading:
        return IMUReading(calibration=(3, 3, 3, 3))

    def close(self) -> None:
        pass


def _s16(lsb: int, msb: int) -> int:
    v = (msb << 8) | lsb
    return v - 65536 if v >= 32768 else v
