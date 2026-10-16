"""step_detect 离线回放测试：合成穿刺曲线触发、静息/无跌落零误报。不访问硬件。"""
import random
import sys
import unittest
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1] / 'code' / 'pc'
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

import step_detect


def sample_puncture(dt, duration=8.0, amp=75000.0, noise=200.0, seed=7):
    """按 mock_slave 的 8s 穿刺循环取样。

    0~2s 静息 -> 2~5s 加载到 amp -> 5.0~5.15s 跌落到 20% -> 逐渐回零。
    噪声默认 200（对应 mock 的 0.02N × 10000 尺度）。
    """
    rng = random.Random(seed)
    samples = []
    t = 0.0
    while t < duration:
        p = t % 8.0
        if p < 2.0:
            base = 0.0
        elif p < 5.0:
            base = amp * (p - 2.0) / 3.0
        elif p < 5.15:
            base = amp * 0.2
        elif p < 6.5:
            base = amp * 0.2 * (1 - (p - 5.15) / 1.35)
        else:
            base = 0.0
        samples.append((t, base + rng.gauss(0, noise)))
        t += dt
    return samples


class StepDetectorTests(unittest.TestCase):
    def test_trigger_on_puncture_at_13hz(self):
        samples = sample_puncture(0.077)
        event, detector = step_detect.detect_step(samples)
        self.assertIsNotNone(event)
        self.assertGreaterEqual(event.time, 5.0)
        self.assertLessEqual(event.time, 5.5)
        self.assertGreater(event.drop, event.threshold)
        self.assertTrue(detector.triggered)

    def test_trigger_latency_at_100hz(self):
        samples = sample_puncture(0.01)
        event, _ = step_detect.detect_step(samples)
        self.assertIsNotNone(event)
        self.assertLessEqual(event.time, 5.35)

    def test_no_trigger_on_rest_noise(self):
        rng = random.Random(3)
        samples = [(i * 0.077, rng.gauss(0, 200.0)) for i in range(400)]  # ~30s
        event, _ = step_detect.detect_step(samples)
        self.assertIsNone(event)

    def test_no_trigger_on_ramp_without_drop(self):
        samples = []
        t = 0.0
        while t < 10.0:
            if t < 1.0:
                value = 0.0
            elif t < 6.0:
                value = 75000.0 * (t - 1.0) / 5.0
            else:
                value = 75000.0
            samples.append((t, value))
            t += 0.05
        event, _ = step_detect.detect_step(samples)
        self.assertIsNone(event)

    def test_short_data_returns_none(self):
        event, detector = step_detect.detect_step([(0.0, 1.0), (0.1, 1.1)])
        self.assertIsNone(event)
        self.assertFalse(detector.baseline_ready)

    def test_event_fields_populated(self):
        samples = sample_puncture(0.077)
        event, _ = step_detect.detect_step(samples)
        self.assertIsNotNone(event)
        self.assertGreater(event.peak, event.value)
        self.assertGreaterEqual(event.baseline_samples, 5)
        self.assertGreater(event.threshold, 0.0)
        self.assertTrue(event.elapsed > 0.0)

    def test_detector_does_not_retrigger(self):
        samples = sample_puncture(0.077)
        event, detector = step_detect.detect_step(samples)
        self.assertIsNotNone(event)
        self.assertIsNone(detector.update(event.time + 0.1, 0.0))


if __name__ == '__main__':
    unittest.main()
