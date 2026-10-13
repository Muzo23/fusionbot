"""Step 13b (offline): protocol 2.1 extension (IMU every step, LiDAR scan at 10 Hz).

    python -m unittest discover -s tests -p "test_step13b*" -v
"""

import struct
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from sim.bridge_script import bridge_script_source
from sim.protocol import PROTOCOL_VERSION, STATE_VALUE_COUNT, decode_state


def packet(ext=None):
    v = [0.0] * STATE_VALUE_COUNT
    v[0], v[1], v[2] = PROTOCOL_VERSION, 7, 0.07
    if ext is not None:
        v += ext
    return struct.pack(f"={len(v)}d", *v)


class TestProtocol21(unittest.TestCase):
    def test_v20_packet_still_accepted(self):
        s = decode_state(packet())
        self.assertEqual(s.imu_count, -1)
        self.assertIsNone(s.imu_gyro_rad_s)
        self.assertIsNone(s.lidar_xyzd)

    def test_imu_without_scan(self):
        s = decode_state(packet([1.0, 42, 0.01, -0.02, 1.2, 0.1, 0.2, 9.81, 41, 0]))
        self.assertEqual(s.imu_count, 42)
        self.assertEqual(s.imu_gyro_rad_s, (0.01, -0.02, 1.2))
        self.assertEqual(s.imu_accel_m_s2, (0.1, 0.2, 9.81))
        self.assertEqual(s.lidar_count, 41)
        self.assertIsNone(s.lidar_xyzd)

    def test_scan_attached(self):
        rays = [0.1, 0.2, 0.0, 0.22, 1.0, 0.0, 0.0, 1.0]
        s = decode_state(packet([1.0, 10, 0, 0, 0, 0, 0, 9.81, 10, 2] + rays))
        self.assertEqual(s.lidar_xyzd, tuple(rays))

    def test_truncated_scan_rejected(self):
        with self.assertRaises(RuntimeError):
            decode_state(packet([1.0, 10, 0, 0, 0, 0, 0, 9.81, 10, 3, 0.1, 0.2, 0.0, 0.22]))

    def test_no_imu(self):
        s = decode_state(packet([1.0, -1, 0, 0, 0, 0, 0, 0, -1, 0]))
        self.assertIsNone(s.imu_gyro_rad_s)
        self.assertEqual(s.lidar_count, -1)

    def test_bridge_reads_imu_and_lidar(self):
        src = bridge_script_source()
        for k in ("signal.fusionbotImuGyro", "signal.fusionbotImuAccel", "signal.fusionbotLidarXYZD",
                  "/FusionBot/mainBase_phy/FusionBotIMU", "/FusionBot/mainBase_phy/fastHokuyo", "LIDAR_DIVIDER = 10"):
            self.assertIn(k, src)


if __name__ == "__main__":
    unittest.main()
