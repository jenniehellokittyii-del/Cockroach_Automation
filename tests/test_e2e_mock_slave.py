"""TCP 端到端：mock_slave（穿刺曲线）→ ForceSensor → StepDetector 触发。无硬件。"""
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PC_DIR = ROOT / 'code' / 'pc'
RS485_DIR = ROOT / 'tools' / 'rs485' / 'source'
for candidate in (PC_DIR, RS485_DIR, RS485_DIR / 'tools'):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

import force_sensor
import step_detect
from mock_slave import MockSlave


class MockSlaveEndToEndTests(unittest.TestCase):
    def test_tcp_puncture_signal_triggers_detector(self):
        slave = MockSlave('127.0.0.1', 0, unit=1, signal='puncture', verbose=False).start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                config = {
                    'mode': 'tcp', 'host': '127.0.0.1', 'tcp_port': slave.port,
                    'unit': 1, 'register': 0x9C40, 'count': 2,
                    'timeout': 0.3, 'poll_interval': 0.002,
                }
                sensor = force_sensor.ForceSensor(
                    config=config, csv_path=Path(tmp) / 'sensor.csv'
                )
                sensor.start()
                try:
                    detector = step_detect.StepDetector()
                    event = None
                    deadline = time.monotonic() + 12.0
                    while time.monotonic() < deadline and event is None:
                        sample = sensor.latest()
                        if sample is not None:
                            event = detector.update(sample[0], sample[2])
                        time.sleep(0.02)
                    self.assertIsNotNone(event, 'mock 穿刺曲线应触发阶跃检测')
                    self.assertGreater(event.drop, event.threshold)
                    self.assertGreaterEqual(sensor.total_samples, 20)
                finally:
                    sensor.close()
        finally:
            slave.stop()


if __name__ == '__main__':
    unittest.main()
