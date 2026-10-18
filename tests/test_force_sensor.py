"""force_sensor 离线测试：32 位组装 / 比例 / 失败计数 / CSV 落盘。不访问硬件。"""
import csv
import sys
import tempfile
import time
import unittest
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1] / 'code' / 'pc'
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

import force_sensor


class FakeLink:
    """假 Modbus 链路：按脚本返回寄存器字，或抛脚本里的异常。"""

    def __init__(self, script):
        self.script = list(script) or [[0, 0]]
        self.calls = 0

    def read_registers(self, addr, count):
        item = self.script[min(self.calls, len(self.script) - 1)]
        self.calls += 1
        if isinstance(item, Exception):
            raise item
        return list(item)

    def open(self):
        pass

    def close(self):
        pass


class CombineWordsTests(unittest.TestCase):
    def test_negative_sample_frame_minus_12(self):
        # 实测帧数据 FF FF FF F4 -> -12
        self.assertEqual(force_sensor.combine_words([0xFFFF, 0xFFF4]), -12)

    def test_limits(self):
        self.assertEqual(force_sensor.combine_words([0x7FFF, 0xFFFF]), 2147483647)
        self.assertEqual(force_sensor.combine_words([0x8000, 0x0000]), -2147483648)

    def test_low_word_first(self):
        self.assertEqual(
            force_sensor.combine_words([0xFFF4, 0xFFFF], high_word_first=False), -12
        )

    def test_unsigned(self):
        self.assertEqual(
            force_sensor.combine_words([0xFFFF, 0xFFF4], signed=False), 4294967284
        )

    def test_bad_length(self):
        with self.assertRaises(ValueError):
            force_sensor.combine_words([1, 2, 3])


class ForceSensorTests(unittest.TestCase):
    def make_sensor(self, tmp, script, **overrides):
        config = {
            'mode': 'serial', 'port': 'COM-TEST', 'register': 0x9C40, 'count': 2,
            'timeout': 0.05, 'poll_interval': 0.002, **overrides,
        }
        link = FakeLink(script)
        sensor = force_sensor.ForceSensor(
            config=config, csv_path=Path(tmp) / 'sensor.csv', link=link
        )
        return sensor, link

    def test_read_once_assembles_signed_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [[0xFFFF, 0xFFF4]])
            raw, value = sensor.read_once()
            self.assertEqual(raw, -12)
            self.assertEqual(value, -12.0)

    def test_read_once_applies_scale(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [[0x0000, 0x0064]], scale=0.01)
            raw, value = sensor.read_once()
            self.assertEqual(raw, 100)
            self.assertAlmostEqual(value, 1.0)

    def test_poll_loop_records_samples_and_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [[0xFFFF, 0xFFF4]] * 200)
            sensor.start()
            time.sleep(0.15)
            sensor.close()
            self.assertGreater(sensor.total_samples, 0)
            self.assertEqual(sensor.total_failures, 0)
            sample = sensor.latest()
            self.assertEqual(sample[1], -12)
            self.assertIsNotNone(sensor.sample_age())
            rows = list(csv.reader((Path(tmp) / 'sensor.csv').open(encoding='utf-8')))
            self.assertEqual(rows[0], ['iso_time', 'monotonic', 'raw', 'value'])
            self.assertGreater(len(rows), 1)

    def test_failures_are_counted(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [RuntimeError('link down')])
            sensor.start()
            time.sleep(0.1)
            sensor.close()
            self.assertGreaterEqual(sensor.consecutive_failures, 1)
            self.assertIn('link down', sensor.last_error)
            self.assertEqual(sensor.total_samples, 0)

    def test_failure_counter_resets_after_recovery(self):
        script = [RuntimeError('x')] * 3 + [[0xFFFF, 0xFFF4]] * 200
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, script)
            sensor.start()
            time.sleep(0.1)
            sensor.close()
            self.assertEqual(sensor.consecutive_failures, 0)
            self.assertGreater(sensor.total_samples, 0)

    def test_save_window_writes_subset(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [[0xFFFF, 0xFFF4]] * 200)
            sensor.start()
            time.sleep(0.1)
            sensor.close()
            center = sensor.latest()[0]
            out = Path(tmp) / 'window.csv'
            rows = sensor.save_window(out, center, before=10.0, after=10.0)
            self.assertGreater(rows, 0)
            window_rows = list(csv.reader(out.open(encoding='utf-8')))
            self.assertEqual(window_rows[0], ['monotonic', 'raw', 'value'])
            self.assertEqual(len(window_rows), rows + 1)

    def test_describe_mentions_link_and_register(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, _ = self.make_sensor(tmp, [[0, 0]])
            text = sensor.describe()
            self.assertIn('COM-TEST', text)
            self.assertIn('0x9C40', text)


if __name__ == '__main__':
    unittest.main()
