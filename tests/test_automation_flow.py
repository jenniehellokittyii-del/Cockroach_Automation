"""流程检查：所有串口、相机与运动均替换为模拟对象。"""
import sys
from contextlib import nullcontext
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code' / 'pc'))

import automation_console as flow
import serial_motion as motion
import vision_workflows as vision


class AutomationFlowTests(unittest.TestCase):
    def setUp(self):
        speed = patch.object(flow, 'positioning_speed', side_effect=lambda c: nullcontext())
        speed.start()
        self.addCleanup(speed.stop)

    def test_home_failure_never_prompts_or_picks(self):
        with patch.object(flow, 'home', return_value=False), \
                patch.object(flow, 'confirm') as confirm, \
                patch.object(flow, 'choose_feed_port') as choose, \
                patch.object(flow, 'pickup_electrode') as pickup:
            self.assertFalse(flow.run_startup(Mock()))
            confirm.assert_not_called()
            pickup.assert_not_called()
            choose.assert_not_called()

    def test_cancel_pickup_never_moves_or_starts_vision(self):
        with patch.object(flow, 'home', return_value=True), \
                patch.object(flow, 'choose_feed_port', return_value=None), \
                patch.object(flow, 'pickup_electrode') as pickup, \
                patch.object(flow, 'visual_position') as visual:
            self.assertFalse(flow.run_startup(Mock()))
            pickup.assert_not_called()
            visual.assert_not_called()

    def test_pickup_failure_never_starts_vision(self):
        with patch.object(flow, 'home', return_value=True), \
                patch.object(flow, 'confirm', return_value=True), \
                patch.object(flow, 'choose_feed_port', return_value=12), \
                patch.object(flow, 'pickup_electrode', return_value=False), \
                patch.object(flow, 'visual_position') as visual:
            self.assertFalse(flow.run_startup(Mock()))
            visual.assert_not_called()

    def test_success_order_and_confirmation_after_lift(self):
        events = []
        with patch.object(flow, 'home', side_effect=lambda c: events.append('home') or True), \
                patch.object(flow, 'confirm', side_effect=lambda p: events.append(p) or True), \
                patch.object(flow, 'choose_feed_port', side_effect=lambda: events.append('choose') or 12), \
                patch.object(flow, 'pickup_electrode', side_effect=lambda c, n: events.append(('pickup_and_lift', n)) or True), \
                patch.object(flow, 'visual_and_drill', side_effect=lambda c: events.append('visual') or True):
            self.assertTrue(flow.run_startup(Mock()))
        self.assertEqual(events, ['home', 'choose', ('pickup_and_lift', 12),
                                  '是否开始视觉识别流程？', 'visual'])

    def test_pickup_uses_selected_port_then_lifts_1000(self):
        controller = Mock(fault=None, home_position={'Z': 20})
        controller.query_position.return_value = {'Z': 11185}
        controller.move_absolute.return_value = True
        with patch.object(motion, 'vision_reference_ready', True), \
                patch.object(motion, 'motion_fault', None), \
                patch.object(flow, 'move_to_feed_port', return_value=True) as pickup:
            self.assertTrue(flow.pickup_electrode(controller, 12))
            pickup.assert_called_once_with(controller, 12)
            controller.move_absolute.assert_called_once_with({'Z': 10185})

    def test_failed_lift_invalidates_reference(self):
        controller = Mock(fault=None, home_position={'Z': 20})
        controller.query_position.return_value = {'Z': 11185}
        controller.move_absolute.return_value = False
        with patch.object(motion, 'vision_reference_ready', True), \
                patch.object(motion, 'motion_fault', None), \
                patch.object(flow, 'move_to_feed_port', return_value=True):
            self.assertFalse(flow.pickup_electrode(controller, 6))
            self.assertFalse(motion.vision_reference_ready)

    def test_vis_skips_home_and_stops_after_one_point(self):
        controller = Mock()
        with patch.object(flow, 'run_visual_head', return_value=True) as visual:
            self.assertTrue(flow.visual_position(controller))
            visual.assert_called_once_with(
                controller.ser, skip_home=True, single_point=True,
                retry_capture=unittest.mock.ANY,
            )
            with patch.object(flow, 'confirm', return_value=True) as confirm:
                self.assertTrue(visual.call_args.kwargs['retry_capture']())
                confirm.assert_called_once_with('识别未完成，是否重新识别？')

    def test_detection_failure_can_retry_without_repositioning(self):
        position = dict(vision.VISION_POS_STEPS, X2=vision.VISION_POS_STEPS['X'])
        calculation = {'target_xy_steps': {'X': 1700, 'Y': -1600},
                       'selected_result_point_index': 1}
        session = {'first_calculation': calculation,
                   'record': {'point_movements': [], 'selected_point_order': [2]},
                   'result_path': Mock(), 'remaining_indices': {0}}
        retry = Mock(return_value=True)
        with patch.object(vision, 'goto_vision_position_without_home', return_value=True) as goto, \
                patch.object(vision, 'capture_and_calculate_vision_target',
                             side_effect=[ValueError('所有帧检测失败'), session]) as capture, \
                patch.object(vision, 'query_positions', return_value=position), \
                patch.object(vision, 'check_vision_xy_move', return_value=({}, {})), \
                patch.object(vision, 'move_continuous', return_value=True) as move, \
                patch.object(motion, 'motion_fault', None), \
                patch.object(motion, 'vision_reference_ready', True):
            self.assertTrue(vision.run_automatic_vision(
                Mock(), skip_home=True, single_point=True, retry_capture=retry))
            self.assertEqual(capture.call_count, 2)
            goto.assert_called_once()
            move.assert_called_once()
            retry.assert_called_once()

    def test_declining_retry_never_moves(self):
        with patch.object(vision, 'goto_vision_position_without_home', return_value=True), \
                patch.object(vision, 'capture_and_calculate_vision_target', side_effect=ValueError('识别失败')) as capture, \
                patch.object(vision, 'move_continuous') as move:
            self.assertFalse(vision.run_automatic_vision(
                Mock(), skip_home=True, retry_capture=lambda: False))
            capture.assert_called_once()
            move.assert_not_called()

    def test_retry_refuses_fault_or_changed_position(self):
        position = dict(vision.VISION_POS_STEPS, X2=vision.VISION_POS_STEPS['X'])
        for fault, pos in [('LIMIT_STOP', position), (None, dict(position, Z=0))]:
            with self.subTest(fault=fault), \
                    patch.object(vision, 'goto_vision_position_without_home', return_value=True), \
                    patch.object(vision, 'capture_and_calculate_vision_target', side_effect=ValueError('识别失败')) as capture, \
                    patch.object(vision, 'query_positions', return_value=pos), \
                    patch.object(motion, 'motion_fault', fault), \
                    patch.object(motion, 'vision_reference_ready', True), \
                    patch.object(vision, 'move_continuous') as move:
                self.assertFalse(vision.run_automatic_vision(
                    Mock(), skip_home=True, retry_capture=lambda: True))
                capture.assert_called_once()
                move.assert_not_called()

    def test_p2_selection(self):
        with patch('builtins.input', side_effect=['p12', 'p2']):
            self.assertEqual(vision.prompt_vision_point_index(
                [[1, 2], [3, 4]], [0, 1], '选择点: '), 1)

    def test_single_point_does_not_prompt_for_another_or_home(self):
        calculation = {'target_xy_steps': {'X': 1700, 'Y': -1600},
                       'selected_result_point_index': 1}
        record = {'point_movements': [], 'selected_point_order': [2]}
        session = {'first_calculation': calculation, 'record': record,
                   'result_path': Mock(), 'remaining_indices': {0, 2}}
        with patch.object(vision, 'goto_vision_position_without_home', return_value=True), \
                patch.object(vision, 'goto_vision_position') as home, \
                patch.object(vision, 'capture_and_calculate_vision_target', return_value=session), \
                patch.object(vision, 'query_positions', return_value=dict(vision.VISION_POS_STEPS)), \
                patch.object(vision, 'check_vision_xy_move', return_value=({}, {})), \
                patch.object(vision, 'move_continuous', return_value=True) as move, \
                patch.object(vision, 'prompt_vision_point_index') as prompt:
            self.assertTrue(vision.run_automatic_vision(Mock(), skip_home=True, single_point=True))
            home.assert_not_called()
            move.assert_called_once_with(unittest.mock.ANY, calculation['target_xy_steps'])
            prompt.assert_not_called()
        self.assertEqual(record['point_movements'][0]['point_number'], 2)


class SelectionAndSpeedTests(unittest.TestCase):
    def test_all_ports_accept_number_or_p_prefix(self):
        for number in range(1, 13):
            for answer in (str(number), f' p{number} '):
                with self.subTest(answer=answer), patch('builtins.input', return_value=answer):
                    self.assertEqual(flow.choose_feed_port(), number)

    def test_invalid_selection_reprompts_and_can_cancel(self):
        with patch('builtins.input', side_effect=['0', 'P13', 'abc', 'Q']):
            self.assertIsNone(flow.choose_feed_port())

    def test_positioning_restores_configured_speed_on_success_and_exception(self):
        for fails in (False, True):
            controller = Mock(fault=None)
            controller.read_response.side_effect = ['SPEED=650.00\r\n', 'SPEED=500.00\r\n']
            with self.subTest(fails=fails), \
                    patch.object(flow.wf, 'load_params', return_value=Mock(normal_speed=500)), \
                    patch.object(motion, 'vision_reference_ready', True):
                def run():
                    with flow.positioning_speed(controller):
                        if fails:
                            raise RuntimeError('movement failed')
                if fails:
                    with self.assertRaisesRegex(RuntimeError, 'movement failed'):
                        run()
                    controller.emergency_stop.assert_called_once()
                    self.assertFalse(motion.vision_reference_ready)
                else:
                    run()
                self.assertEqual(controller.send.call_args_list,
                                 [unittest.mock.call('SPEED 650'), unittest.mock.call('SPEED 500')])

    def test_speed_ack_failure_never_enters_movement(self):
        for response in ('', 'ERR BUSY', 'SPEED=500.00\r\n'):
            controller = Mock(fault=None)
            controller.read_response.side_effect = [response, 'SPEED=500.00\r\n']
            move = Mock()
            with self.subTest(response=response), \
                    patch.object(flow.wf, 'load_params', return_value=Mock(normal_speed=500)), \
                    patch.object(motion, 'vision_reference_ready', True):
                with self.assertRaises(RuntimeError):
                    with flow.positioning_speed(controller):
                        move()
                move.assert_not_called()
                self.assertFalse(motion.vision_reference_ready)
                controller.emergency_stop.assert_called()
                self.assertEqual(controller.send.call_args_list[-1], unittest.mock.call('SPEED 500'))

    def test_restore_failure_stops_and_invalidates_reference(self):
        controller = Mock(fault=None)
        controller.read_response.side_effect = ['SPEED=650.00\r\n', '']
        with patch.object(flow.wf, 'load_params', return_value=Mock(normal_speed=500)), \
                patch.object(motion, 'vision_reference_ready', True):
            with self.assertRaises(RuntimeError):
                with flow.positioning_speed(controller):
                    pass
            self.assertFalse(motion.vision_reference_ready)
            controller.emergency_stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
