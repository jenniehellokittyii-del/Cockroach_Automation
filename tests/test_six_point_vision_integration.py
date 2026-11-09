"""Checks that the new points, compensation files, and existing entrypoints agree."""
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PC = ROOT / 'code' / 'pc'
sys.path.insert(0, str(PC))
sys.path.insert(0, str(ROOT / 'code' / 'vision'))

import dachuang_strategy
import motion_client
import serial_motion
import settings
import vision_workflows


class SixPointVisionIntegrationTests(unittest.TestCase):
    def test_runtime_assets_and_calibration_paths_exist(self):
        self.assertTrue(settings.VISION_PIXEL_CALIBRATION.is_file())
        self.assertTrue(settings.VISION_TOOL_CALIBRATION.is_file())
        self.assertTrue(settings.VISION_POINT_COMPENSATION.is_file())
        self.assertTrue(settings.VISION_HEAD_POINT_COMPENSATION.is_file())
        self.assertTrue((ROOT / 'code' / 'vision' / 'dachuang_strategy.py').is_file())
        self.assertTrue((ROOT / 'code' / 'vision' / 'dachuang' /
                         'six_point_geometry_20261106.npz').is_file())

    def test_drill_console_motion_entry_and_shared_home_state_remain(self):
        self.assertTrue(callable(motion_client.handle_motion_command))
        self.assertTrue(hasattr(serial_motion, 'vision_reference_ready'))
        self.assertTrue(hasattr(serial_motion, 'motion_fault'))

    def test_new_head_point_four_compensation_and_legacy_difference(self):
        new = json.loads(settings.VISION_HEAD_POINT_COMPENSATION.read_text(encoding='utf-8'))
        old_path = ROOT / 'config' / 'vision_head_point_compensation.json'
        old = json.loads(old_path.read_text(encoding='utf-8'))
        self.assertEqual(new['source_point_numbers'], [1, 2, 3, 4])
        self.assertEqual(new['workflow_local_point_numbers'], [1, 2, 3, 4])
        self.assertEqual(old['correction_steps_by_point_number']['4']['X'], -38)
        self.assertEqual(old['correction_steps_by_point_number']['4']['Y'], 106)
        p4 = new['correction_steps_by_point_number']['4']
        self.assertEqual(p4['X'] - old['correction_steps_by_point_number']['4']['X'], 38)
        self.assertEqual(p4['Y'] - old['correction_steps_by_point_number']['4']['Y'], -26)

    def test_vis_selects_source_p5_p6_and_uses_local_compensation(self):
        points = np.array([[10, 10], [20, 20], [30, 30], [40, 40],
                           [50, 50], [60, 60]], dtype=float)
        with patch.object(dachuang_strategy, 'detect_six_points',
                          return_value=(points, {'method': 'TEST'})):
            result = dachuang_strategy.detect_multiframe(
                [np.zeros((100, 100, 3), dtype=np.uint8)])
        self.assertEqual(result['source_point_numbers'], [5, 6])
        self.assertEqual(result['points'], [[50.0, 50.0], [60.0, 60.0]])

        calibration = json.loads(
            settings.VISION_PIXEL_CALIBRATION.read_text(encoding='utf-8'))
        tool = json.loads(
            settings.VISION_TOOL_CALIBRATION.read_text(encoding='utf-8'))
        reference_pixel = calibration['reference_p2_center_px']
        compensation = json.loads(
            settings.VISION_POINT_COMPENSATION.read_text(encoding='utf-8'))
        self.assertEqual(compensation['source_point_numbers'], [5, 6])
        self.assertEqual(compensation['workflow_local_point_numbers'], [1, 2])
        selected = vision_workflows.calculate_vision_target(
            {'points': [reference_pixel, reference_pixel]}, calibration, tool,
            point_index=1, point_compensation=compensation)
        self.assertEqual(selected['point_compensation_steps'], {'X': -35, 'Y': 170})

    def test_selected_head_p4_uses_new_compensation_table(self):
        calibration = json.loads(
            settings.VISION_PIXEL_CALIBRATION.read_text(encoding='utf-8'))
        tool = json.loads(
            settings.VISION_TOOL_CALIBRATION.read_text(encoding='utf-8'))
        reference_pixel = calibration['reference_p2_center_px']
        points = [[0, 0], [1, 1], [2, 2], reference_pixel]
        compensation = json.loads(
            settings.VISION_HEAD_POINT_COMPENSATION.read_text(encoding='utf-8'))
        result = vision_workflows.calculate_vision_target(
            {'points': points}, calibration, tool,
            point_index=3, point_compensation=compensation)
        self.assertEqual(result['selected_result_point_index'], 3)
        self.assertEqual(result['point_compensation_steps'], {'X': 0, 'Y': 80})
        before = result['target_before_point_compensation_steps']
        after = result['target_xy_steps']
        self.assertEqual(after['X'] - before['X'], 0)
        self.assertEqual(after['Y'] - before['Y'], 80)

    def test_geometry_model_declares_symmetric_pairs_and_six_labels(self):
        model_path = (ROOT / 'code' / 'vision' / 'dachuang' /
                      'six_point_geometry_20261106.npz')
        with np.load(model_path, allow_pickle=False) as model:
            self.assertTrue(bool(model['symmetric_pairs'].item()))
            self.assertEqual(model['geometry_template'].shape, (6, 2))
            self.assertEqual(model['points'].shape, (6, 2))


if __name__ == '__main__':
    unittest.main()
