# 测试报告

## 1. 硬件连通性

ESP32-S3 通过 I2C 总线连接三类传感器，实测扫描结果包含：

- HMC5883L：`0x1E`
- MPU6050/MPU6500 兼容 IMU：`0x68`
- BMP280/BME280：`0x76`

GNSS 模块通过 UART 接入，RX/TX 分别接 GPIO16/GPIO17，PPS 接 GPIO21。

## 2. 标定结果

### 2.1 加速度计

采用六位置 12 参数模型进行加速度计标定。代表性结果：

- 标定后平均向量误差：`1.798 mg`
- 标定后最大向量误差：`2.120 mg`
- 目标阈值：约 `2 mg`

数据文件位于 `data/calibration/accel_6pos_20260705_114101.csv`。

### 2.2 磁力计

采用椭球拟合估计硬铁中心和软铁补偿矩阵。代表性结果：

- 原始磁场模长均值/标准差：`46.285 / 3.303 uT`
- 补偿后磁场模长均值/标准差：`52.248 / 1.892 uT`

数据文件位于 `data/calibration/mag_rotate_20260705_135521.csv`。

### 2.3 陀螺仪 Allan 方差

完成 300 s 级静态陀螺仪采集和 Allan 方差分析，结果见：

- `data/calibration/gyro_allan_20260705_171933_allan.csv`
- `figures/gyro_allan_deviation.png`

## 3. 姿态融合与可视化

实现互补滤波和 Mahony 滤波两种传统姿态融合算法。实时网页显示能够显示 Roll、Pitch、Yaw、气压高度、磁场模长和样本计数。

相关图表：

- `figures/attitude_algorithm_compare.png`
- `figures/system_architecture.png`

## 4. AI 增强算法

完成 IMU 静态数据 1D 卷积去噪，并与传统低通滤波、1D 卷积 + 一阶 Kalman 混合结构对比。代表性结果：

- 加速度计平均 SNR 提升：传统低通 `9.686 dB`，1D 卷积 `18.121 dB`，1D 卷积 + Kalman `18.687 dB`
- 陀螺仪平均 SNR 提升：传统低通 `12.299 dB`，1D 卷积 `15.697 dB`，1D 卷积 + Kalman `15.837 dB`

## 5. 气压高度实验

BMP280 楼梯高度实验中，人工测量楼层高度 `3.95 m`，BMP280 测量高度约 `3.980 m`，绝对误差约 `0.030 m`，相对误差约 `0.76%`。

相关文件：

- `data/fusion_comparison/bmp_stair_height_20260706_summary.csv`
- `figures/bmp_stair_height_20260706.png`

## 6. GNSS 与时间同步

完成 GNSS NMEA 解析、PPS 时间同步记录和 GNSS/INS 松耦合静态分析。由于台式电脑无法移动，户外连续轨迹叠加未作为最终必做结果提交。

相关文件：

- `data/fusion_comparison/gps_nmea_parse_20260706_171832.csv`
- `data/performance/time_sync_static_20260706_105553.csv`
- `figures/gps_nmea_parse_20260706_171832_gnss_ins_20260706_172456.png`

## 7. 性能测试

板端性能统计证明高频采样/更新达到要求：

- 主循环更新频率：约 `229.617 Hz`
- IMU 更新频率：约 `229.617 Hz`
- 互补滤波更新频率：约 `229.617 Hz`
- Mahony 更新频率：约 `229.617 Hz`

数据文件位于 `data/performance/perf_highrate_20260707_090954.txt`。
