# Project Audit

本文件记录从原始目录 `E:\桌面\归档文件\驭远杯` 整理为 GitHub 项目时的判断。

## 原始目录观察

- 原始目录不是 Git 仓库。
- `wpa_supplicant.conf` 含明文 Wi-Fi 信息，不应上传 GitHub。
- `WAVE_ROVER_FACTORY.zip` 和解压后的烧录工具体积较大，不适合放入代码仓库。
- 原始 PDF/PPT 资料保留在本地归档目录，仓库只放用于说明项目流程的局部截图。

## 主程序判断

`芋圆杯.txt` 是最终代码，但转移时出现了编码损坏：

- 中文注释变成乱码。
- 部分代码被乱码注释吞入同一行，例如 `BASE_SPEED`、`CROSS_DETECT_THRESHOLD`、`num_cross = 11`。
- 存在明显拼写错误 `Truse`。

整理后的 `src/rover_main.py` 基于 `芋圆杯.txt` 修复得到，保留最终控制逻辑，并补充结构化注释。修复后已通过 `python -m py_compile` 语法检查。

## 公开仓库策略

- `src/rover_main.py`：最终主程序，纳入仓库。
- `manual_tests/`：底盘、循迹、避障、摄像头等单功能测试脚本，纳入仓库用于复现实验过程。
- `experiments/`：历史组合方案和参考实现，纳入仓库但不作为主入口。
- `docs/`：图文版赛道说明和整理记录，纳入仓库。
- 敏感配置、运行状态、大型工具包、完整规则 PDF/PPT 不纳入仓库。

