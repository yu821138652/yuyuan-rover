# YuYuan Rover

第十二届驭远杯小车项目整理版。项目代码运行在 Raspberry Pi 小车上，完成 AprilTag 目标识别、红外循迹、十字路口计数、超声波避障、河谷段稳定行驶和终点卸货等任务。

![赛道平面示意图](docs/assets/track_overview.png)

## 项目亮点

- 5 段任务状态机：初始循迹、中段循迹、避障、河谷、卸货
- 状态持久化：使用 `mission_state.json` 保存检查点、目标编号、阶段和十字路口计数
- 多传感器融合：四路红外循迹、超声波测距、AprilTag 视觉识别
- 运行安全：异常退出或手动中断时停止电机并释放串口/GPIO
- 图文说明：见 [docs/TRACK_ROUTE.md](docs/TRACK_ROUTE.md)

## 项目结构

```text
.
├── src/
│   └── rover_main.py              # 最终主程序
├── manual_tests/                  # 单功能手动测试脚本
├── experiments/                   # 历史组合方案和参考实现
├── docs/
│   ├── TRACK_ROUTE.md             # 图文版赛道路线说明
│   └── assets/                    # 文档配图
├── requirements.txt
└── README.md
```

## 运行环境

目标平台为 Raspberry Pi。完整运行需要连接真实硬件：

- 串口底盘控制器
- 四路红外循迹传感器
- 三个超声波传感器
- 舵机卸货机构
- Raspberry Pi Camera

主要 Python 依赖见 `requirements.txt`。其中 `picamera2`、`lgpio` 等通常需要在 Raspberry Pi OS 上安装。

## 快速运行

在 Raspberry Pi 上进入项目目录：

```bash
pip install -r requirements.txt
python src/rover_main.py
```

重置任务状态后重新开始：

```bash
rm -f mission_state.json
python src/rover_main.py
```

从检查点恢复时，可以手动编辑 `mission_state.json`：

```json
{
  "startpoint": "S2",
  "target_id": 0,
  "flag": 2,
  "num_cross": 8
}
```

## 代码来源与整理说明

原始文件夹中 `芋圆杯.txt` 是最终代码，但转移时混入了中文乱码、不可打印字符和少量被注释吞掉的代码行。本仓库的 `src/rover_main.py` 基于它修复得到：

- 恢复被乱码注释吞掉的关键语句，例如 `BASE_SPEED`、`CROSS_DETECT_THRESHOLD`、`num_cross = 11`
- 修复明显拼写错误 `Truse -> True`
- 清理损坏的中文注释，并补充结构化中文注释
- 使用 `python -m py_compile` 做静态语法检查

## 不纳入 GitHub 的内容

以下内容保留在原始归档目录，不建议上传到公开仓库：

- `wpa_supplicant.conf`：包含 Wi-Fi 信息
- `WAVE_ROVER_FACTORY/` 与 `WAVE_ROVER_FACTORY.zip`：出厂烧录工具和二进制文件
- 赛事 PDF/PPT 原件：仓库只保留用于说明的局部截图
- `mission_state.json`：运行时状态文件

