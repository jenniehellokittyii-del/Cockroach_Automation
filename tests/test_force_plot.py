"""CSV 观察窗口回归检查；不连接硬件、不创建实际窗口。"""
import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('force_plot', ROOT / 'tools/drill_plot/force_plot.py')
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)
sys.path.insert(0, str(ROOT / 'code/pc'))
import automation_console as flow


class ForcePlotTests(unittest.TestCase):
    def test_existing_latest_and_subsequent_round(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            old = directory / 'sensor_1.csv'
            latest = directory / 'sensor_2.csv'
            old.write_text('old')
            latest.write_text('latest')
            os.utime(old, ns=(1_000_000_000, 1_000_000_000))
            os.utime(latest, ns=(2_000_000_000, 2_000_000_000))
            known = set()
            self.assertEqual(plot.newest_new_csv(directory, known), latest)
            self.assertIsNone(plot.newest_new_csv(directory, known))
            new = directory / 'sensor_3.csv'
            new.write_text('new')
            self.assertEqual(plot.newest_new_csv(directory, known), new)

    def test_watch_switches_after_first_round(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / 'sensor_old.csv').write_text(
                'monotonic,raw,value\n1,999,999\n')
            fig = Mock(number=1)
            ax, ax2 = Mock(transAxes=object()), Mock(transAxes=object())
            ax.plot.return_value = (Mock(),)
            ax2.plot.return_value = (Mock(),)
            reads = []
            rounds = [0]
            def pause(interval):
                rounds[0] += 1
                (directory / f'sensor_{rounds[0]}.csv').write_text(
                    'monotonic,raw,value\n100,10,10\n101,8,8\n')
            original = plot.read_csv
            def read(path):
                reads.append(path.name)
                return original(path)
            with patch.object(sys, 'argv', ['force_plot', '--watch', tmp, '--show-now']), \
                 patch.object(plot.plt, 'subplots', return_value=(fig, (ax, ax2))), \
                 patch.object(plot.plt, 'fignum_exists', side_effect=lambda n: rounds[0] < 3), \
                 patch.object(plot.plt, 'pause', side_effect=pause), \
                 patch.object(plot.plt, 'close'), patch.object(plot, 'read_csv', side_effect=read):
                plot.main()
            self.assertEqual(reads, ['sensor_1.csv', 'sensor_2.csv'])

    def test_watch_refreshes_growing_current_file_and_marks_idle(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            path = directory / 'sensor_new.csv'
            fig = Mock(number=1)
            ax, ax2 = Mock(transAxes=object()), Mock(transAxes=object())
            line, data_state = Mock(), Mock()
            ax.plot.return_value = (line,)
            ax.text.side_effect = [Mock(), data_state]
            ax2.plot.return_value = (Mock(),)
            rounds = [0]

            def pause(interval):
                rounds[0] += 1
                if rounds[0] == 1:
                    path.write_text('monotonic,raw,value\n100,10,10\n')
                elif rounds[0] == 2:
                    with path.open('a') as stream:
                        stream.write('101,12,12\n')

            with patch.object(sys, 'argv', ['force_plot', '--watch', tmp, '--show-now']), \
                 patch.object(plot.plt, 'subplots', return_value=(fig, (ax, ax2))), \
                 patch.object(plot.plt, 'fignum_exists', side_effect=lambda n: rounds[0] < 4), \
                 patch.object(plot.plt, 'pause', side_effect=pause), \
                 patch.object(plot.plt, 'close'), \
                 patch.object(plot.time, 'monotonic', side_effect=[0, 0, 1, 1, 4]):
                plot.main()
            self.assertIn(unittest.mock.call([0.0], [10.0]), line.set_data.call_args_list)
            self.assertIn(unittest.mock.call([0.0, 1.0], [10.0, 12.0]), line.set_data.call_args_list)
            self.assertIn(unittest.mock.call('实时数据更新中'), data_state.set_text.call_args_list)
            data_state.set_text.assert_called_with('数据暂未更新，保留本轮曲线')

    def test_csv_format_matches_sensor(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sensor.csv'
            path.write_text('iso_time,monotonic,raw,value\nnow,100.5,-3,-0.3\nnow,101.5,7,0.7\n')
            self.assertEqual(plot.read_csv(path), ([0.0, 1.0], [-3.0, 7.0]))

    def test_launcher_uses_independent_process_and_absolute_paths(self):
        with patch.object(flow.subprocess, 'Popen') as launch:
            self.assertIs(flow.start_force_plot(), launch.return_value)
        args = launch.call_args.args[0]
        self.assertEqual(args[0], sys.executable)
        self.assertIn(str(ROOT / 'tools/drill_plot/force_plot.py'), args)
        self.assertIn(str(ROOT / 'runtime/motion_logs'), args)

    def test_window_failure_does_not_abort_automation(self):
        with patch.object(flow.subprocess, 'Popen', side_effect=OSError('test failure')):
            self.assertIsNone(flow.start_force_plot())

    def test_main_stops_window_and_motion_on_cancel(self):
        controller, process = Mock(), Mock()
        with patch.object(flow, 'MotionController', return_value=controller), \
             patch.object(flow, 'start_force_plot', return_value=process), \
             patch.object(flow, 'stop_force_plot') as stop, \
             patch.object(flow, 'run_startup', return_value=False):
            self.assertEqual(flow.main(['--serial-port', 'MOCK']), 1)
        controller.emergency_stop.assert_called_once()
        controller.close.assert_called_once()
        stop.assert_called_once_with(process)


if __name__ == '__main__':
    unittest.main()
