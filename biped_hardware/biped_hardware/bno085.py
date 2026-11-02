"""BNO085 I2C driver using Adafruit's BNO08x library."""

from dataclasses import dataclass


@dataclass
class IMUReading:
    # ROS-compatible order: w, x, y, z
    quaternion: tuple = (1.0, 0.0, 0.0, 0.0)
    accel: tuple = (0.0, 0.0, 9.81)
    gyro: tuple = (0.0, 0.0, 0.0)
    calibration: tuple = (0, 0, 0, 0)


class BNO085:
    def __init__(self, i2c_bus=1, address=0x4A, mode="imuplus"):
        self.bus_num = int(i2c_bus)
        self.address = int(address)
        self._i2c = None
        self._sensor = None
        self._last = IMUReading()

    def initialize(self):
        import board
        import busio
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_ROTATION_VECTOR,
        )

        # This implementation currently supports the Pi's primary I2C bus.
        if self.bus_num != 1:
            raise ValueError(
                "BNO085 Blinka driver currently requires i2c_bus=1 "
                "on this Raspberry Pi"
            )

        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        try:
            self._sensor = BNO08X_I2C(
                self._i2c, address=self.address
            )
            self._sensor.enable_feature(BNO_REPORT_ACCELEROMETER, 10000)
            self._sensor.enable_feature(BNO_REPORT_GYROSCOPE, 10000)
            self._sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR, 10000)
        except Exception:
            self.close()
            raise

    def read(self):
        if self._sensor is None:
            return self._last

        try:
            accel = self._sensor.acceleration
            gyro = self._sensor.gyro
            q = self._sensor.quaternion  # x, y, z, w

            if accel is not None and gyro is not None and q is not None:
                x, y, z, w = q
                self._last = IMUReading(
                    quaternion=(w, x, y, z),
                    accel=tuple(float(v) for v in accel),
                    gyro=tuple(float(v) for v in gyro),
                    # BNO085 calibration status isn't mapped here yet.
                    calibration=(0, 0, 0, 0),
                )
        except Exception:
            # Keep the last valid reading; do not invent new sensor values.
            pass

        return self._last

    def close(self):
        self._sensor = None
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
            self._i2c = None


class MockBNO085:
    def initialize(self):
        pass

    def read(self):
        return IMUReading(calibration=(3, 3, 3, 3))

    def close(self):
        pass
