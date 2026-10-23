# 钻孔 D1-D3 提交版说明（2026-10-23）

> 版本：`2026-10-23-D1D3`  
> 用途：先交一版可运行代码 + 文档 + 日志 + 数据 + 视频。  
> 当前配置：`drill_duty=0`，钻头不转；D1 只下 Z；D3 为电极移动。  
> 工具：T1 = 电极头，T2 = 钻孔头。

---

## 1. 先看哪份

| 对象 | 看什么 |
|---|---|
| 队长/现场 | `docs\指令_简单版.md` |
| 研发/排错 | `docs\指令_复杂版.md` |
| 参数 | `docs\参数表.md` |
| 测试数据 | `docs\测试记录_20261023.md` + `data\` |
| 原始记录 | `motion_logs\` |
| 完整视频 | `视频\完整测试_20261023.mp4` |

---

## 2. 包内结构

```text
钻孔D1-D3交付_20261023/
├─ pc\                              可运行代码 + 标定文件
│  ├─ full_flow.py                  D1-D3 控制台
│  ├─ motion_client.py              串口/转盘/SAFE/ECAL/D3
│  ├─ force_sensor.py               力传感器
│  ├─ step_detect.py                阶跃检测
│  ├─ drill_cycle.py                下 Z/钻孔状态机
│  ├─ DM_CAN.py                     转盘电机
│  ├─ vis_head.py                   仅 import 依赖；D1-D3 不用视觉
│  ├─ xy_coordinate_transform.py    坐标转换依赖
│  ├─ force_sensor.json
│  ├─ safe_z.json
│  ├─ drill_start_z.json
│  ├─ electrode_calibration.json
│  ├─ xy_pixel_calibration.json
│  └─ vision_position.json
├─ config\
│  ├─ drill_params.json             参数记录
│  ├─ force_sensor.json
│  └─ calibration_20261023\         标定备份
├─ docs\
│  ├─ 指令_简单版.md                你桌面改的 11 步操作卡
│  ├─ 指令_复杂版.md                完整流程/命令/排错，不重复给 Z
│  ├─ 参数表.md                     非 Z 参数 + 文件位置
│  └─ 测试记录_20261023.md          结果概览
├─ data\
│  ├─ drill_results_20261023.csv    30 条 D1/DC 结果
│  └─ sensor_summary_20261023.csv   37 个传感器日志摘要
├─ motion_logs\                     全部 208 个原始日志
├─ 视频\
│  └─ 完整测试_20261023.mp4         完整测试视频（4.23 MB）
├─ run_drill_test.bat
├─ requirements.txt
├─ README_交付.md
└─ README_PR结构.md                 ← 本文件
```

---

## 3. PR / 仓库结构

当前 GitHub 上的 PR #3 是**旧版**，不是本次最新 D1-D3。  
本次先交的是上面这个压缩包版本。

### 3.1 代码进入仓库时的建议位置（A 方案）

```text
code/pc/full_flow.py
code/pc/motion_client.py
code/pc/force_sensor.py
code/pc/step_detect.py
code/pc/drill_cycle.py
code/pc/DM_CAN.py
code/pc/vis_head.py
code/pc/xy_coordinate_transform.py
config/force_sensor.json
run_drill_test.bat
requirements.txt
```

包内路径与仓库路径对应关系：

| 包内 | 仓库 |
|---|---|
| `pc\full_flow.py` | `code/pc/full_flow.py` |
| `pc\motion_client.py` | `code/pc/motion_client.py` |
| `pc\force_sensor.py` | `code/pc/force_sensor.py` |
| `pc\step_detect.py` | `code/pc/step_detect.py` |
| `pc\drill_cycle.py` | `code/pc/drill_cycle.py` |
| `pc\DM_CAN.py` | `code/pc/DM_CAN.py` |
| `pc\vis_head.py` | `code/pc/vis_head.py` |
| `pc\xy_coordinate_transform.py` | `code/pc/xy_coordinate_transform.py` |
| `pc\force_sensor.json` | `config/force_sensor.json` |
| `run_drill_test.bat` | `run_drill_test.bat` |

### 3.2 视频 / 数据 / 日志

本次提交包已全部包含：

```text
motion_logs\                 全部原始日志
视频\完整测试_20261023.mp4   完整测试视频
data\drill_results...        整理后的 D1 结果
data\sensor_summary...       传感器摘要
```

代码 PR 建议只放代码；如果队长要求视频/数据也进仓库，可放：

```text
docs/media/完整测试_20261023.mp4
data/20261023/
```

### 3.3 旧 PR #3 的情况

```text
分支：feature/drill-final
提交：9283a81
状态：旧版
差异：没有最新 SAFE=0、drill_duty=0、D3=500、最新 full_flow
```

拉取 PR #3 时注意不要覆盖本包内 `pc\` 的最新代码。

---

## 4. 当前运行参数

```text
T1 = 电极头
T2 = 钻孔头
drill_duty = 0（不转钻头）
D3 速度 = 500 步/秒
进给 = 500 步/秒
安全限深 = 2500 步 = 25mm
抬升 = 2000 步 = 20mm
超时 = 60s
ECAL 最大 = 10000 步 = 100mm
```

Z 相关参数（安全高度、钻孔起点、offset、insert）已测好，保存在：

```text
pc\safe_z.json
pc\drill_start_z.json
pc\electrode_calibration.json
config\calibration_20261023\
```

文档不重复写具体 Z 数值。`Z_STOP` 每次 D1 自动更新，不固定保存。

---

## 5. 怎么运行

```text
双击 run_drill_test.bat
```

或：

```powershell
cd /d "本包目录"
python pc\full_flow.py
```

简单流程：

```text
TZ → TE → D1 → D2 → D3 → Q
```

当前 D1 的“开钻”是动作名称，实际**不转钻头**，只下 Z。

---

## 6. 日志与数据

```text
motion_logs\20261023_*.log             当天全部串口/流程日志
motion_logs\sensor_20261023_*.csv      37 个传感器日志
motion_logs\drill_event_20261023_*.csv 18 个触发窗口
data\drill_results_20261023.csv        30 条 D1/DC 结果
data\sensor_summary_20261023.csv       传感器摘要
```

早期传感器无变化的日志也保留，没有删除。

### 数据记录约定

这些 JSON 只保存“当前最新值/配置”，不是历史：

```text
force_sensor.json            配置
safe_z.json                  最后一次 SAFE
drill_start_z.json           最后一次 DRILL_START
electrode_calibration.json   最后一次电极标定
xy_pixel_calibration.json    视觉配置
vision_position.json         视觉配置
```

以后每次实验：

```text
原始数据自动写入 motion_logs\
人工记录追加到 data\run_records.csv
覆盖标定前备份到 config\calibration_日期\
```

---

## 7. 提交/发送文案

可以直接这样发：

```text
这是 2026-10-23 钻孔 D1-D3 提交版。
包内含：可运行代码、简单/复杂指令、参数表、全部日志、
D1 数据、传感器摘要、完整测试视频，以及全部标定文件。
当前配置不转钻头，D1 只下 Z；Z 参数已测好，直接使用标定文件。
完整内容见压缩包和 README_PR结构.md。
```

---

## 8. 版本状态

```text
提交包：已生成
简单版：已用你桌面修改后的版本替换
复杂版/参数表/README/测试记录：已按“Z 已测好、不重复给 Z”重写
测试脚本/代码：未再改动
PR：本次先交压缩包版本；GitHub PR #3 仍是旧版
```
