"""合并控制台的路由和坐标基准检查，不连接实际设备。"""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code' / 'pc'))

import drill_console as console
import motion_client as cli
import serial_motion as motion


class CombinedConsoleTests(unittest.TestCase):
    def test_home_waits_for_completion_and_invalidates_old_drill(self):
        with patch.object(console.wf, 'LAST_DRILL_RESULT', {'z_stop': 9000}), \
                patch.object(motion, 'vision_reference_ready', False), \
                patch.object(motion, 'motion_fault', 'old limit'), \
                patch.object(cli, 'send') as send, \
                patch.object(cli, 'read_response', return_value='HOME DONE') as read:
            console._manual(object(), 'HOME')
            send.assert_called_once_with(unittest.mock.ANY, 'HOME')
            self.assertEqual(read.call_args.kwargs['markers'], ('HOME DONE', 'FAIL'))
            self.assertTrue(motion.vision_reference_ready)
            self.assertIsNone(motion.motion_fault)
            self.assertIsNone(console.wf.LAST_DRILL_RESULT)

    def test_failed_home_and_partial_home_do_not_authorize_fast_vis(self):
        for command, response in [('HOME', 'HOME FAIL'), ('HX', 'HOMED X')]:
            with self.subTest(command=command), \
                    patch.object(motion, 'vision_reference_ready', True), \
                    patch.object(cli, 'send'), \
                    patch.object(cli, 'read_response', return_value=response):
                console._manual(object(), command)
                self.assertFalse(motion.vision_reference_ready)

    def test_zero_invalidates_old_drill_and_reference(self):
        with patch.object(console.wf, 'LAST_DRILL_RESULT', {'z_stop': 9000}), \
                patch.object(motion, 'vision_reference_ready', True), \
                patch.object(cli, 'send'), patch.object(cli, 'read_response', return_value='ZERO'):
            console._manual(object(), 'ZERO')
            self.assertIsNone(console.wf.LAST_DRILL_RESULT)
            self.assertFalse(motion.vision_reference_ready)

    def test_vis_aliases_use_original_modes(self):
        ser = object()
        with patch.object(cli, 'run_automatic_vision') as abdomen, \
                patch.object(cli, 'run_visual_head') as head:
            console._manual(ser, 'VIS')
            abdomen.assert_called_with(ser)
            console._manual(ser, 'VISF')
            abdomen.assert_called_with(ser, skip_home=True)
            console._manual(ser, 'VIS_HEAD')
            head.assert_called_with(ser)
            console._manual(ser, 'VIS_HEADF')
            head.assert_called_with(ser, skip_home=True)

    def test_invalid_pwm_is_not_sent(self):
        with patch.object(cli, 'send') as send:
            console._manual(object(), 'PWM 101')
            send.assert_not_called()

    def test_jog_and_diagnostics_share_connection(self):
        ser = object()
        with patch.object(cli, 'jog_mode') as jog, \
                patch.object(cli, 'send') as send, \
                patch.object(cli, 'read_response', return_value=''):
            console._manual(ser, 'JOG')
            jog.assert_called_once_with(ser)
            console._manual(ser, 'DIAG')
            send.assert_called_once_with(ser, 'DIAG')

    def test_one_console_routes_motion_vision_and_all_drill_stages(self):
        ser, tt = Mock(), Mock()
        with patch.object(sys, 'argv', ['drill_console.py', '--serial-port', 'COM20']), \
                patch('builtins.input', side_effect=['HOME', 'VIS_FAST', 'PWM 80', 'D1', 'D2', 'D3', 'Q']), \
                patch.object(console, 'open_verified_arduino', return_value=ser) as connect, \
                patch.object(console, 'connect_turntable', return_value=tt), \
                patch.object(console, '_state_text', return_value='state'), \
                patch.object(console, 'print_help'), \
                patch.object(console, 'send'), patch.object(console, 'read_response', return_value=''), \
                patch.object(cli, 'send') as send, \
                patch.object(cli, 'read_response', return_value='HOME DONE'), \
                patch.object(cli, 'run_automatic_vision') as visual, \
                patch.object(console.wf, 'drill_stage') as d1, \
                patch.object(console.wf, 'swap_stage') as d2, \
                patch.object(console.wf, 'electrode_stage') as d3, \
                patch.object(motion, 'vision_reference_ready', False), \
                patch.object(motion, 'motion_fault', None), \
                patch.object(console.wf, 'LAST_DRILL_RESULT', None):
            console.main()
            connect.assert_called_once_with('COM20')
            visual.assert_called_once_with(ser, skip_home=True)
            send.assert_any_call(ser, 'DRILL PWM 80')
            d1.assert_called_once_with(ser, tt)
            d2.assert_called_once_with(ser, tt)
            d3.assert_called_once_with(ser, tt)
            ser.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
