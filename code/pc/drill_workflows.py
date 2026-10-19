# -*- coding: utf-8 -*-
"""钻孔流程（DC 命令）：传感器阶跃 -> 停钻 -> 抬升固定距离。

与 `vision_workflows.py` 对应：这里是串口/传感器编排层，
核心状态机在 `drill_cycle.py`，传感器读取在 `force_sensor.py`。

前置条件：XY 已对准孔位、Z 已在待钻高度（人工或视觉流程到位），本命令不移动 XY。
传感器串口独立于 Arduino：使用前先关闭 RS485 GUI（串口独占）。
"""

from datetime import datetime

import serial_motion as motion
from serial_motion import send, read_response, query_positions, wait_positions
from settings import DRILL_EVENT_LOG_DIR
from force_sensor import ForceSensor, load_config as load_force_sensor_config
from drill_cycle import DrillParams, run_drill_cycle

# 钻孔参数（进给/限深/抬升/超时），现场按需调整
DRILL_PARAMS = DrillParams()


class SerialDrillIO:
    """把 serial_motion 的串口函数包装成 drill_cycle 需要的 IO 接口。"""

    def __init__(self, ser):
        self.ser = ser

    def send(self, line):
        send(self.ser, line)

    def read_response(self, markers=None, timeout=0.3):
        return read_response(self.ser, markers=markers, timeout=timeout)

    def query_positions(self, timeout=1.0):
        return query_positions(self.ser, timeout=timeout)

    def wait_z(self, target, timeout=20.0):
        return wait_positions(self.ser, {'Z': target}, timeout=timeout)

    def fault(self):
        return motion.motion_fault

    def clear_fault(self):
        motion.motion_fault = None

    def log(self, message):
        print(message)
        motion.log_io('EVT', message)


def run_automatic_drill(ser):
    """DC：传感器阶跃 → 停钻 → 抬升固定距离（不自动移动 XY）。

    传感器用 config/force_sensor.json 配置；启动失败时不做任何运动。
    """
    config = load_force_sensor_config()
    sensor = ForceSensor(config=config)
    try:
        sensor.start()
    except Exception as exc:
        print(f'DC: 传感器启动失败，未开始钻孔：{exc}')
        print('  检查：传感器 COM 口/接线/波特率；并确认 RS485 GUI 已关闭（串口独占）')
        return False
    print(f'DC: 传感器已连接（{sensor.describe()}）')
    print(f'DC: 传感器数据记录到 {sensor.csv_path}')
    try:
        io = SerialDrillIO(ser)
        result = run_drill_cycle(io, sensor, params=DRILL_PARAMS, log=io.log)
        event = result.get('trigger')
        if isinstance(event, dict) and event.get('time') is not None:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            window_path = DRILL_EVENT_LOG_DIR / f'drill_event_{stamp}.csv'
            rows = sensor.save_window(
                window_path, float(event['time']), before=2.0, after=2.0
            )
            print(f'DC: 触发前后 {rows} 个传感器样本已保存 {window_path}')
        if result['status'] == 'BREAKTHROUGH' and result['lifted']:
            print(f"DC: 完成——钻深 {result['depth_steps']} 步，"
                  f"已抬升 {DRILL_PARAMS.lift_steps} 步"
                  f"（Z={result['z_lift_target']}）；换头请手动进行")
        else:
            print(f"DC: 未完成——status={result['status']}，"
                  f"lifted={result['lifted']}，请人工确认机构状态后再操作")
        return result['status'] == 'BREAKTHROUGH'
    finally:
        sensor.close()
