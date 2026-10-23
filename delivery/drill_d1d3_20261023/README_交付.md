# 钻孔 D1-D3 交付包（2026-10-23）

> 默认前提：所有 Z 参数已经测好并保存在标定文件里。  
> 当前配置：钻头不转（drill_duty=0），D1 只下 Z；D2/ECAL/D3 为电极流程。  
> 工具：T1 = 电极头，T2 = 钻孔头。

## 1. 怎么开始

双击：

```text
run_drill_test.bat
```

或命令行：

```powershell
cd /d "本包目录"
python pc\full_flow.py
```

## 2. 先看哪份文档

| 文档 | 给谁看 |
|---|---|
| `docs\指令_简单版.md` | 队长/现场操作，只看这一页 |
| `docs\指令_复杂版.md` | 研发/排错，完整流程和说明 |
| `docs\参数表.md` | 非 Z 参数和文件位置 |
| `docs\测试记录_20261023.md` | 今天的结果和日志说明 |
| `docs\数据记录说明.md` | JSON/日志/后续数据记录约定 |

## 3. 当前状态

```text
T1 = 电极头
T2 = 钻孔头
drill_duty = 0（不转钻头）
D3 速度 = 500 步/秒
Z 参数：使用已保存的标定文件，不在文档中重复给数值
```

## 4. 包内结构

```text
钻孔D1-D3交付_20261023/
├─ pc\                  运行代码 + 标定文件
├─ config\              参数记录、传感器配置、标定备份
├─ docs\                简单版/复杂版/参数表/测试记录
├─ motion_logs\         全部原始日志（208 个文件）
├─ data\                整理后的 D1 结果和传感器摘要
├─ 视频\                完整测试视频
├─ run_drill_test.bat
├─ requirements.txt
└─ README_交付.md
```

## 5. 标定文件

全部保留，分别在：

```text
pc\safe_z.json
pc\drill_start_z.json
pc\electrode_calibration.json
config\calibration_20261023\
```

> 这些 JSON 保存的是“当前最新值/配置”，不是历史。以后每次实验：
> - 原始数据自动进入 `motion_logs\`；
> - 人工记录追加到 `data\run_records.csv`；
> - 覆盖标定前先备份到 `config\calibration_日期\`。
> 详见 `docs\数据记录说明.md`。

## 6. 重要说明

- `Z_STOP` 每次 D1 自动更新，不需要手动维护；D3 用当前会话的 Z_STOP。
- Z 参数已测好，标准流程不需要重新输入 SAFE/DRILLSTART/OFFSET 的具体数值。
- 日志全部保存在 `motion_logs\`；早期传感器无变化的日志也保留。
- 完整测试视频：`视频\完整测试_20261023.mp4`。

## 7. 依赖

见 `requirements.txt`；传感器需要 pymodbus，USB-CAN 转盘需要 pyserial。
