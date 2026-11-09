"""No physical devices are opened by these checks."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code' / 'pc'))

import motion_client as cli
import serial_motion as motion
import vision_workflows as vision


class MotionSafetyTests(unittest.TestCase):
    def test_command_validation(self):
        self.assertEqual(motion.normalize_motion_command('PWM 80'), 'DRILL PWM 80')
        self.assertEqual(motion.normalize_motion_command('E'), 'E')
        for command in ('PWM -1', 'PWM 101', 'PWM fast'):
            with self.subTest(command=command), self.assertRaises(ValueError):
                motion.normalize_motion_command(command)

    def test_home_failure_never_moves(self):
        with patch.object(motion, 'send'), patch.object(motion, 'read_response', return_value='HOME FAIL'), \
                patch.object(motion, 'move_continuous') as move:
            self.assertFalse(motion.goto_vision_position(object()))
            move.assert_not_called()

    def test_fast_mode_requires_known_home(self):
        with patch.object(motion, 'vision_reference_ready', False), \
                patch.object(motion, 'query_positions') as query:
            self.assertFalse(motion.goto_vision_position_without_home(object()))
            query.assert_not_called()

    def test_xy_guard_checks_fault_and_distance(self):
        target = {'X': 100, 'Y': 0}
        current = {'X': 0, 'X2': 0, 'Y': 0, 'Z': vision.VISION_POS_STEPS['Z']}
        with patch.object(vision, 'query_positions', return_value=current), \
                patch.object(motion, 'motion_fault', 'LIMIT_STOP'):
            with self.assertRaises(ValueError):
                vision.check_vision_xy_move(object(), target)
        with patch.object(vision, 'query_positions', return_value=current), \
                patch.object(motion, 'motion_fault', None):
            with self.assertRaises(ValueError):
                vision.check_vision_xy_move(
                    object(), {'X': vision.VISION_MAX_XY_MOVE_STEPS + 1, 'Y': 0})

    def test_cli_resets_shared_home_state(self):
        with patch('builtins.input', side_effect=['HOME', 'exit']), \
                patch.object(cli, 'send'), patch.object(cli, 'read_response', return_value='HOME FAIL'):
            motion.vision_reference_ready = True
            cli.cmd_mode(object(), None)
            self.assertFalse(motion.vision_reference_ready)


if __name__ == '__main__':
    unittest.main()
