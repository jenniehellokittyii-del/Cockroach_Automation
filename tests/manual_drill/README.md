# 钻孔现场脚本（手动硬件检查）

本目录是钻孔流程的现场工具，**主程序不依赖它们**，运行前请阅读并有人监护。

## `drill_console.py` —— 现场钻孔控制台（推荐）

手动控制 + `RUN` 开始钻孔（内部使用 `code/pc/drill_workflows.py` 的串口适配）：

```powershell
python tests\manual_drill\drill_console.py
```

也可以直接双击：

- `run_drill_test.bat` —— real 模式（需要传感器）
- `run_drill_test_dry.bat` —— dry 模式（不接传感器，钻到限深停）

- 自动连接 Arduino（校验固件 + 发 E）与力传感器（默认 `COM6` / 19200；可在 `--sensor-port` 覆盖）
- `drill>` 控制台：`X/Y/Z/B` 相对移动、`G0/M0`、`POS`、`DRILL ...`、`E`
- `RUN` 按当前参数钻孔（断开/超时/限深都会 `STOP` 并抬升）
- `CFG` 现场修改进给/限深/抬升/超时与检测参数；`S/F/M` 看力值；`ZERO` 坐标清零（不动机械）；`Q` 退出
- 负 Z 保护：控制台内部把 `ZDOWN` 换成相对下行（`Z 100000`），负坐标也能正常下钻

```powershell
python tests\manual_drill\drill_console.py --dry        # 不接传感器，只验证运动/停/抬升
python tests\manual_drill\drill_console.py --sensor-port COM6 --feed 500 --depth 2000
```

## `drill_field_test.py` —— 菜单版备用工具

与上面的控制台功能类似，命令为 `s/m/z/r/d/l/p/c/h/e/q`（按 `?` 看帮助）；适合临时调参。
它使用标准 `SerialDrillIO`（固件 `ZDOWN`），负 Z 时会由保护逻辑拒绝 `RUN`。

```powershell
python tests\manual_drill\drill_field_test.py
```

## `scan_sensor.py` —— 传感器诊断

寄存器快照 + 按压测试（判断力是否进入变送器）：

```powershell
python tests\manual_drill\scan_sensor.py
```

## 演示视频

`docs/videos/钻孔成功_20261017.mp4` —— 现场完整循环：下钻 → 刺穿触发 → 停钻 → 抬升。

## 安全

- 全程有人监护，手边可断电；`E` 随时急停（固件同时关钻头）
- 自动抬升限制在 `Z ≥ 0`；顶部 Z 限位开关需单独检查
- **不要**在无人监护时运行

详细参数见 `docs/钻孔参数表.md`；完整方案见 `docs/传感器钻孔控制工作方案_v2.md`。
