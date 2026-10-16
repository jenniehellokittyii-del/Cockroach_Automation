"""drill_cycle 离线测试：五场景（正常/限深/超时/断线/LIMIT_STOP）命令顺序与安全路径。无硬件。"""
import sys
import time
import unittest
from pathlib import Path


PC_DIR = Path(__file__).resolve().parents[1] / 'code' / 'pc'
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

import drill_cycle
from step_detect import StepEvent


class StubDetector:
    """按调用次数触发，彻底与时间/曲线解耦。"""

    def __init__(self, trigger_after=None):
        self.trigger_after = trigger_after
        self.calls = 0

    def update(self, t, value):
        self.calls += 1
        if self.trigger_after is not None and self.calls >= self.trigger_after:
            return StepEvent(
                time=t, value=value, peak=value + 1000.0, peak_time=t - 0.1,
                drop=1000.0, threshold=500.0, baseline_mean=0.0, baseline_std=1.0,
                baseline_samples=10, elapsed=t,
            )
        return None


class FakeIO:
    """假串口：记录命令并按命令前缀返回回包；位置按步进推进。"""

    def __init__(self, z_start=1000, step=25, lift_result=True,
                 drill_on_ok=True, zdown_ok=True, fault_after_queries=None,
                 fault_text='LIMIT_STOP pin=27 ms=1 pos=1 target=2'):
        self.commands = []
        self.z = z_start
        self.step = step
        self.lift_result = lift_result
        self.drill_on_ok = drill_on_ok
        self.zdown_ok = zdown_ok
        self.query_count = 0
        self.fault_after_queries = fault_after_queries
        self.fault_text = fault_text
        self._fault = None
        self.cleared = 0
        self.lift_targets = []

    def send(self, line):
        self.commands.append(line)

    def read_response(self, markers=None, timeout=0.3):
        if not self.commands:
            return ''
        last = self.commands[-1]
        up = last.upper()
        if up.startswith('SPEED'):
            return f'SPEED={last.split()[1]}'
        if up == 'DRILL ON':
            return 'DRILL ON DUTY=50 FREQ=25000 DIR=CW' if self.drill_on_ok else ''
        if up == 'DRILL STATUS':
            return 'DRILL OFF DUTY=0 FREQ=DC DIR=CW'
        if up == 'ZDOWN':
            return 'MOVE Z DOWN CONTINUOUS; E=STOP' if self.zdown_ok else ''
        if up == 'STOP':
            return 'STOP'
        if up.startswith('Z -'):
            return f'MOVE {last}'
        return ''

    def query_positions(self, timeout=1.0):
        self.query_count += 1
        if (self.fault_after_queries is not None
                and self.query_count > self.fault_after_queries):
            self._fault = self.fault_text
        z = self.z
        self.z += self.step
        return {'X': 1666, 'X2': 1666, 'Y': -1603, 'Z': z, 'B': 0}

    def wait_z(self, target, timeout):
        self.lift_targets.append(target)
        return self.lift_result

    def fault(self):
        return self._fault

    def clear_fault(self):
        self.cleared += 1
        self._fault = None


class FakeSensor:
    """假传感器：默认一直有新鲜样本；fail_from_call 后模拟断线。"""

    def __init__(self, fail_from_call=None):
        self.calls = 0
        self.fail_from_call = fail_from_call
        self.consecutive_failures = 0
        self.last_error = None
        self.total_samples = 0

    def latest(self):
        self.calls += 1
        if self.fail_from_call is not None and self.calls >= self.fail_from_call:
            self.consecutive_failures = 99
            self.last_error = 'link down'
            return None
        self.total_samples += 1
        return (time.monotonic(), 1, 42.0)


BASE_PARAMS = dict(
    feed_speed=40.0, normal_speed=500.0, safe_depth_steps=500, lift_steps=1000,
    timeout_seconds=2.0, pos_poll_interval=0.005, sensor_stale_seconds=0.5,
    sensor_fail_limit=10, loop_wait=0.001, lift_timeout_seconds=1.0,
)


def run_cycle(io, sensor, detector, **overrides):
    params = drill_cycle.DrillParams(**{**BASE_PARAMS, **overrides})
    return drill_cycle.run_drill_cycle(
        io, sensor, params=params, detector=detector, log=lambda message: None
    )


class DrillCycleTests(unittest.TestCase):
    def test_breakthrough_stops_then_lifts(self):
        io = FakeIO(step=2)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(trigger_after=5))
        cmds = [c.upper() for c in io.commands]
        self.assertEqual(result['status'], 'BREAKTHROUGH')
        self.assertTrue(result['lifted'])
        self.assertTrue(result['drill_off_verified'])
        self.assertEqual(result['z_lift_target'], result['z_stop'] - 1000)
        # 顺序：进给 -> 钻头 -> ZDOWN；STOP -> 复核/恢复 -> 抬升
        self.assertLess(cmds.index('SPEED 40'), cmds.index('DRILL ON'))
        self.assertLess(cmds.index('DRILL ON'), cmds.index('ZDOWN'))
        self.assertLess(cmds.index('STOP'), cmds.index('DRILL STATUS'))
        self.assertLess(cmds.index('STOP'), cmds.index('Z -1000'))
        self.assertLess(cmds.index('SPEED 500'), cmds.index('Z -1000'))
        self.assertEqual(io.lift_targets, [result['z_lift_target']])
        self.assertIsNotNone(result['trigger'])
        self.assertGreaterEqual(result['depth_steps'], 0)

    def test_depth_limit_aborts_and_lifts(self):
        io = FakeIO(step=100)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'DEPTH_LIMIT')
        self.assertGreaterEqual(result['depth_steps'], 500)
        self.assertTrue(result['lifted'])
        self.assertIn('Z -1000', [c.upper() for c in io.commands])

    def test_timeout_aborts_and_lifts(self):
        io = FakeIO(step=0)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(None), timeout_seconds=0.1)
        self.assertEqual(result['status'], 'TIMEOUT')
        self.assertTrue(result['lifted'])

    def test_sensor_fault_after_start_aborts_and_lifts(self):
        io = FakeIO(step=0)
        sensor = FakeSensor(fail_from_call=6)
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'SENSOR_FAULT')
        cmds = [c.upper() for c in io.commands]
        self.assertIn('STOP', cmds)
        self.assertTrue(result['lifted'])

    def test_limit_stop_aborts_and_lifts(self):
        io = FakeIO(step=5, fault_after_queries=3)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'LIMIT_STOP')
        self.assertIn('LIMIT_STOP', result['detail'])
        self.assertTrue(result['lifted'])
        self.assertTrue(result['drill_off_verified'])

    def test_drill_on_failure_still_stops_and_restores_speed(self):
        io = FakeIO(drill_on_ok=False)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'DRILL_ON_FAILED')
        cmds = [c.upper() for c in io.commands]
        self.assertIn('STOP', cmds)
        self.assertIn('SPEED 500', cmds)

    def test_no_zdown_aborts(self):
        io = FakeIO(zdown_ok=False)
        sensor = FakeSensor()
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'NO_ZDOWN')
        self.assertIn('STOP', [c.upper() for c in io.commands])

    def test_sensor_missing_before_start_does_nothing(self):
        io = FakeIO()
        sensor = FakeSensor(fail_from_call=1)
        result = run_cycle(io, sensor, StubDetector(None))
        self.assertEqual(result['status'], 'SENSOR_FAULT')
        self.assertEqual(io.commands, [])
        self.assertFalse(result['lifted'])
        self.assertEqual(io.lift_targets, [])


if __name__ == '__main__':
    unittest.main()
