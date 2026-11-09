"""自动化头部识别的重试和单点定位，不接硬件。"""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code' / 'pc'))
import serial_motion as motion
import vision_workflows as vision


class HeadAutomationTests(unittest.TestCase):
    def test_head_retry_then_p2_moves_once_without_abdomen_or_home(self):
        points = [[10, 10], [20, 20], [30, 30], [40, 40]]
        position = dict(vision.VISION_POS_STEPS, X2=vision.VISION_POS_STEPS['X'])
        target = {axis: position[axis] for axis in ('X', 'Y')}
        retry = Mock(return_value=True)
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(vision, 'VISION_HEAD_OUTPUT_ROOT', Path(directory)), \
                patch.object(vision, 'goto_vision_position_without_home', return_value=True) as goto, \
                patch.object(vision, 'goto_vision_position') as home, \
                patch.object(vision, 'capture_and_detect_head', side_effect=[ValueError('头部未识别'), points]) as capture, \
                patch.object(vision, 'capture_and_calculate_vision_target') as abdomen, \
                patch.object(vision, 'query_positions', return_value=position), \
                patch.object(vision, 'calculate_vision_target', return_value={'target_xy_steps': target}) as calculate, \
                patch.object(vision, 'check_vision_xy_move', return_value=(position, {'X': 0, 'Y': 0})), \
                patch.object(vision, 'move_continuous', return_value=True) as move, \
                patch.object(motion, 'motion_fault', None), \
                patch.object(motion, 'vision_reference_ready', True), \
                patch('builtins.input', side_effect=['p2']) as choice:
            self.assertTrue(vision.run_visual_head(
                Mock(), skip_home=True, single_point=True, retry_capture=retry))
            self.assertEqual(capture.call_count, 2)
            retry.assert_called_once()
            goto.assert_called_once()
            home.assert_not_called()
            abdomen.assert_not_called()
            choice.assert_called_once()
            self.assertEqual(calculate.call_args.kwargs['point_index'], 1)
            move.assert_called_once()
            self.assertEqual(calculate.call_args.kwargs['point_compensation'],
                             vision.json.loads(vision.VISION_HEAD_POINT_COMPENSATION.read_text(encoding='utf-8')))

    def test_declining_head_retry_does_not_move(self):
        with patch.object(vision, 'goto_vision_position_without_home', return_value=True), \
                patch.object(vision, 'capture_and_detect_head', side_effect=ValueError('头部未识别')) as capture, \
                patch.object(vision, 'move_continuous') as move:
            self.assertFalse(vision.run_visual_head(
                Mock(), skip_home=True, single_point=True, retry_capture=lambda: False))
            capture.assert_called_once()
            move.assert_not_called()

    def test_head_retry_refuses_motion_fault(self):
        with patch.object(vision, 'goto_vision_position_without_home', return_value=True), \
                patch.object(vision, 'capture_and_detect_head', side_effect=ValueError('头部未识别')) as capture, \
                patch.object(vision, 'query_positions', return_value=dict(vision.VISION_POS_STEPS)), \
                patch.object(motion, 'vision_reference_ready', True), \
                patch.object(motion, 'motion_fault', 'LIMIT_STOP'), \
                patch.object(vision, 'move_continuous') as move:
            self.assertFalse(vision.run_visual_head(
                Mock(), skip_home=True, single_point=True, retry_capture=lambda: True))
            capture.assert_called_once()
            move.assert_not_called()

    def test_head_cannot_skip_home_without_known_reference(self):
        with patch.object(motion, 'vision_reference_ready', False), \
                patch.object(vision, 'capture_and_detect_head') as capture:
            self.assertFalse(vision.run_visual_head(Mock(), skip_home=True, single_point=True))
            capture.assert_not_called()


if __name__ == '__main__':
    unittest.main()
