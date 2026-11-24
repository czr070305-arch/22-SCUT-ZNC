# capture/ —— 上位机调试脚本

配合 `code/` 固件用。Python 3 + pyserial / numpy / opencv。

## 结构

```
capture/
├── recv_images.py      采图（常用）
├── telemetry.py        实时遥测（常用）
├── diag_serial.py      串口诊断（常用）
│
├── core/               核心模块 ★ 这些被 analysis/ 引用，别单独挪走
│   ├── vision_sim.py      视觉算法 Python 移植版（被 13 个脚本引用）
│   ├── vision_sim_v2.py   另一个独立版本（不依赖 v1）
│   ├── labels.py          图集标注读写（被 7 个引用）
│   ├── island.py          环岛识别
│   └── viz.py             可视化画图
│
├── analysis/           离线评测和分析
│   ├── eval.py / eval_full.py      评测通过率
│   ├── check_new.py / check_dataset.py  图集质量检查
│   ├── find_tears.py              找撕裂帧
│   ├── overview.py                图集总览拼图
│   ├── err_breakdown.py           拆解 err：dx / 斜率项 各占多少
│   ├── trace_corner.py            追踪直角弯判定过程
│   ├── undist_check.py            畸变校正检查
│   ├── binaview.py                看分割结果
│   ├── compare_sheet.py           多套参数横向对比
│   ├── report.py / robust.py      报告 / 鲁棒性测试
│   ├── tune.py                    参数扫描
│   └── check_c.py                 检查 C 源文件编码/括号
│
├── logs/               遥测日志（跑出来的，不进 git）
├── archive/            22 个一次性补丁脚本（历史遗留，不用管）
└── README.md           本文件
```

## 怎么跑

**采图**（在工程根目录）：
```bash
py -3 capture/recv_images.py -p COM11 -o dataset
```

**实时遥测**：
```bash
py -3 capture/telemetry.py -p COM11 --log capture/logs/run.csv
```

**离线分析**（任意目录都能跑，脚本自己找 dataset）：
```bash
py -3 capture/analysis/err_breakdown.py
py -3 capture/analysis/eval_full.py
```

## 数据在哪

图集在工程根目录的 `dataset/`（不在 capture 里）。
`analysis/` 下的脚本已经改成用绝对路径找它，不受当前目录影响。

## 产物命名

所有运行产物的文件名以 `_` 开头（如 `_overview.png`），
已被 `.gitignore` 的 `capture/_*` 规则排除，不会进仓库。想清理直接删。
