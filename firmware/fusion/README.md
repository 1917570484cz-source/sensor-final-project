# fusion

本目录包含姿态融合、GNSS/INS 和可视化相关脚本：

- 互补滤波与 Mahony 滤波主实现位于 `../main.c`
- `analyze_attitude_compare.py`：姿态算法结果对比
- `analyze_eskf_static.py`：静态姿态精度分析
- `gnss_ins_loose_coupling.py`：GNSS/INS 松耦合静态分析
- `realtime_attitude_web.py`：串口实时姿态网页显示
