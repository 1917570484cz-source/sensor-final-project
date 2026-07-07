# 论文图表清单

本清单按论文正文位置整理。已有文件可直接插入 Word；标记为“待补”的项目需要硬件截图或用户手工插入。GPS 户外轨迹叠加为选做扩展项，本文最终不作为核心结果。

## 正文图

| 图号建议 | 图题建议 | 文件 | 状态 | 放置位置 |
|---|---|---|---|---|
| 图 1 | 系统总体架构框图 | `figures/system_architecture.svg` | 已有 | 2.1 总体架构 |
| 图 2 | PCB 原理图 | 待插入截图 | 待补 | 2.3 PCB 原理图设计 |
| 图 3 | PCB 布局与传感器位置 | 待插入截图 | 待补 | 2.4 PCB 布局走线 |
| 图 4 | 加速度计六位置标定前后误差对比 | `figures/accel_calibration_errors.svg` | 已有 | 4.2 |
| 图 5 | 磁力计校准前后三维散点图 | `figures/mag_3d_scatter_before_after.svg` | 已有 | 4.3 |
| 图 6 | 磁力计校准前后模长对比 | `figures/mag_norm_before_after.svg` | 已有 | 4.3 |
| 图 7 | 航向残差补偿前后误差对比 | `figures/heading_residual_errors.svg` | 已有 | 4.4 |
| 图 8 | 陀螺仪 Allan 方差曲线 | `figures/gyro_allan_deviation.svg` | 已有 | 4.5 / 6.2 |
| 图 9 | 互补滤波与 Mahony 滤波姿态输出对比 | `figures/attitude_algorithm_compare.svg` | 已有 | 5.1 |
| 图 10 | BMP280 楼梯高度实验曲线 | `figures/bmp_stair_height_20260706.svg` | 已有 | 6.4 |
| 图 11 | GPS 静态定位散点图 | `figures/gps_static_scatter_20260706.svg` | 已有 | 6.5 |
| 图 12 | GPS 静态高度变化 | `figures/gps_static_altitude_20260706.svg` | 已有 | 6.5 |
| 图 13 | GNSS/INS 松耦合静态结果 | `figures/gps_nmea_parse_20260706_171832_gnss_ins_20260706_172456.svg` | 已有 | 5.3 |
| 图 14 | GPS 户外轨迹图 | 户外有效定位轨迹未形成 | 选做未完成 | 6.5 |
| 图 15 | GPS/气压高度融合曲线 | 无有效户外 GPSLOG，未生成 | 选做未完成 | 6.5 |
| 图 16 | 实时姿态 Web 页面截图 | 浏览器截图 | 待补截图 | 5.1 / 数据可视化 |

## 正文表

| 表号建议 | 表题建议 | 数据来源 | 状态 |
|---|---|---|---|
| 表 1 | 传感器选型及接口 | 正文整理 | 已写入正文 |
| 表 2 | 加速度计六位置标定结果 | `data/accel_calibration_20260705_114101.txt` | 已写入正文 |
| 表 3 | 磁力计椭球标定结果 | `data/mag_calibration_20260705_134935.txt` | 已写入正文 |
| 表 4 | 陀螺仪 Allan 方差结果 | `data/gyro_allan_20260705_171933_allan.txt` | 已写入正文 |
| 表 5 | 标定与补偿效果汇总 | 多个实验结果 | 已写入正文 |
| 表 6 | 姿态算法静态稳定性对比 | `data/eskf_static_level_20260706_095447_eskf_static_20260706_095751.txt` | 已写入正文 |
| 表 7 | AI 去噪平均 SNR 提升 | `data/eskf_static_level_20260706_095447_ai_imu_denoise_20260706_103101.txt` | 已写入正文 |
| 表 8 | GPS NMEA 解析质量统计 | `data/gps_nmea_parse_gps_parsed_20260706_171832_summary_20260706_171929.txt` | 可补进正文 |
| 表 9 | PPS 时间同步统计 | `data/gps_pps_sync2_20260706_155714_gps_pps_sync_20260706_155824.txt` | 可补进正文 |
| 表 10 | 固件 CPU 占用统计 | `data/perf_cpu_20260706_094755.txt` | 已写入正文 |

## 需要后续补图的命令

正式 GPSLOG dump 后生成轨迹：

```powershell
python analysis\plot_gps_track.py --csv data\gpslog_dump_xxx.csv
```

正式 GPSLOG dump 后生成 GPS/气压高度融合：

```powershell
python analysis\analyze_height_fusion.py --gps-csv data\gpslog_dump_xxx.csv
```

如需重新生成已有基础图：

```powershell
python analysis\generate_figures.py
```
