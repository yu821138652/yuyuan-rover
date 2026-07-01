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



