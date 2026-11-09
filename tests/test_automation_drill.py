"""头部选点接入 D1–D3 的流程检查，禁止连接实体硬件。"""
import sys
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code' / 'pc'))
import automation_console as flow
import serial_motion as motion


class AutomationDrillTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.controller = Mock(home_position={'Z': 20})
        self.position = {'X': 825, 'X2': 825, 'Y': -1332, 'Z': 8908, 'B': 0}
        self.safe_position = dict(self.position, Z=5908)
        self.query = self.stack.enter_context(patch.object(
            motion, 'query_positions', side_effect=[self.position, self.safe_position]))
        self.stack.enter_context(patch.object(motion, 'vision_reference_ready', True))
        self.stack.enter_context(patch.object(motion, 'motion_fault', None))
        self.stack.enter_context(patch.object(flow.wf, 'LAST_DRILL_RESULT', None))
        self.stack.enter_context(patch.object(flow.wf, 'load_state', return_value={
            'safe_z_steps': 5908, 'drill_start_z': 7908, 'offset_steps': 0, 'insert_steps': 1850}))
        self.stack.enter_context(patch.object(flow.wf, 'load_params', return_value=Mock(drill_duty=0)))
        self.move = self.stack.enter_context(patch.object(motion, 'move_continuous', return_value=True))
        self.confirm = self.stack.enter_context(patch.object(flow, 'confirm', return_value=True))
        self.tt = Mock(enabled=True, hold_fault=None)
        self.factory = self.stack.enter_context(patch.object(flow, 'Turntable', return_value=self.tt))

    def test_lift_then_confirmation_then_zero_enable_and_full(self):
        events = []
        self.move.side_effect = lambda *args: events.append('lift') or True
        self.confirm.side_effect = lambda *args: events.append('confirm') or True
        self.tt.set_zero.side_effect = lambda: events.append('TZ')
        self.tt.enable.side_effect = lambda: events.append('TE')
        with patch.object(flow.wf, 'run_full', side_effect=lambda *args: events.append('FULL') or True) as full:
            self.assertTrue(flow.drill_after_selection(self.controller))
            self.assertEqual(events, ['lift', 'confirm', 'TZ', 'TE', 'FULL'])
            self.move.assert_called_once_with(self.controller.ser, {'Z': 5908})
            full.assert_called_once_with(self.controller.ser, self.tt)
            self.tt.close.assert_called_once()

    def test_cancel_does_not_connect_or_run_drill(self):
        self.confirm.return_value = False
        with patch.object(flow.wf, 'run_full') as full:
            self.assertTrue(flow.drill_after_selection(self.controller))
            self.factory.assert_not_called()
            full.assert_not_called()

    def test_failed_lift_never_enables_turntable(self):
        self.move.return_value = False
        self.assertFalse(flow.drill_after_selection(self.controller))
        self.confirm.assert_not_called()
        self.factory.assert_not_called()

    def test_position_change_during_confirmation_blocks_drill(self):
        self.query.side_effect = [self.position, dict(self.safe_position, X=0)]
        self.assertFalse(flow.drill_after_selection(self.controller))
        self.factory.assert_not_called()

    def test_invalid_visual_position_blocks_lift(self):
        self.query.side_effect = [dict(self.position, Z=0)]
        self.assertFalse(flow.drill_after_selection(self.controller))
        self.move.assert_not_called()
        self.factory.assert_not_called()

    def test_turntable_failure_stops_and_closes_without_drilling(self):
        self.tt.enable.side_effect = RuntimeError('反馈超时')
        with patch.object(flow.wf, 'run_full') as full:
            self.assertFalse(flow.drill_after_selection(self.controller))
            full.assert_not_called()
            self.controller.emergency_stop.assert_called_once()
            self.tt.close.assert_called_once()

    def test_d1_failure_blocks_d2_and_d3(self):
        with patch.object(flow.wf, 'drill_stage', return_value={'status': 'DEPTH_LIMIT'}), \
                patch.object(flow.wf, 'swap_stage') as d2, \
                patch.object(flow.wf, 'electrode_stage') as d3:
            self.assertFalse(flow.drill_after_selection(self.controller))
            d2.assert_not_called()
            d3.assert_not_called()
            self.controller.emergency_stop.assert_called_once()
            self.tt.close.assert_called_once()

    def test_real_full_dispatches_d1_d2_d3_in_order(self):
        events = []
        result = {'status': 'BREAKTHROUGH', 'lifted': True,
                  'drill_off_verified': True, 'safe_returned': True}
        with patch.object(flow.wf, 'drill_stage', side_effect=lambda *a, **kw: events.append('D1') or result), \
                patch.object(flow.wf, 'swap_stage', side_effect=lambda *a: events.append('D2') or True), \
                patch.object(flow.wf, 'electrode_stage', side_effect=lambda *a: events.append('D3') or True):
            self.assertTrue(flow.drill_after_selection(self.controller))
            self.assertEqual(events, ['D1', 'D2', 'D3'])

    def test_failed_vision_never_starts_drill(self):
        with patch.object(flow, 'visual_position', return_value=False), \
                patch.object(flow, 'drill_after_selection') as drill:
            self.assertFalse(flow.visual_and_drill(self.controller))
            drill.assert_not_called()


if __name__ == '__main__':
    unittest.main()
